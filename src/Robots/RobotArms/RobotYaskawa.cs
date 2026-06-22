using static System.Math;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

public class RobotYaskawa : RobotArm
{
    internal RobotYaskawa(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.Yaskawa, payload, basePlane, baseMesh, joints) { }

    private protected override NumericalKinematics CreateSolver() => new(this);
    public override double DegreeToRadian(double degree, int i) => degree * (PI / 180.0);
    public override double RadianToDegree(double radian, int i) => radian * (180.0 / PI);
    protected override double[] DefaultAlpha => [0, -HalfPI, 0, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, -HalfPI, 0, HalfPI, HalfPI, 0];
    protected override int[] DefaultSign => [1, 1, 1, 1, 1, 1];
}

internal class YaskawaKinematics(RobotArm robot) : RobotKinematics(robot)
{
    readonly double[] _start = [0, HalfPI, HalfPI, 0, 0, PI];
    readonly double[] _signs = [1, -1, -1, 1, -1, 1];

    /// <summary>
    /// Code adapted from https://github.com/Jmeyer1292/opw_kinematics
    /// </summary>
    protected override double[] InverseKinematics(Transform t, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors)
    {
        bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
        bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
        bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

        errors = [];
        bool isUnreachable = false;
        bool isSingularity = false;

        // Hardcoded physical dimensions of the Yaskawa Motoman MH6 based on the OPW convention
        var a1 = 0.0;      // No parallel forward offset between J1 and J2 centerlines
        var a2 = -155.0;   // Vertical shoulder/elbow offset step
        var b = 0.0;      // No side-to-side track offset
        var c1 = 450.0;    // Height from the base alignment to the J2 shoulder center
        var c2 = 614.0;    // Length of the lower arm link (J2 center to J3 center)
        var c3 = 640.0;    // Length of the upper arm boom (J3 center to J5 wrist center)
        var c4 = 95.0;     // Reach from the J5 center line forward to the tool flange face

        // Step 1: Locate the Wrist Center Point (P-Point) by back-stepping along the flange normal vector
        var flange = t.ToPlane();
        Point3d c = flange.Origin - c4 * flange.Normal;
        var nx1 = Sqrt(c.X * c.X + c.Y * c.Y - b * b) - a1;

        var tmp1 = Atan2(c.Y, c.X);
        var tmp2 = Atan2(b, nx1 + a1);

        var joints = new double[6];

        // Step 2: Solve Joint 1 (Base Rotation)
        joints[0] = !shoulder ? tmp1 - tmp2 : tmp1 + tmp2 - PI;

        var tmp3 = c.Z - c1;
        var kappa_2 = a2 * a2 + c3 * c3;
        var c2_2 = c2 * c2;

        var s1_2 = nx1 * nx1 + tmp3 * tmp3;
        var tmp4 = nx1 + 2.0 * a1;
        var s2_2 = tmp4 * tmp4 + tmp3 * tmp3;

        // Step 3: Solve Joint 2 (Shoulder Angle)
        if (!shoulder)
        {
            var s1 = Sqrt(s1_2);
            var tmp5 = s1_2 + c2_2 - kappa_2;
            var tmp13 = Acos(tmp5 / (2.0 * s1 * c2));
            var tmp14 = Atan2(nx1, c.Z - c1);

            if (double.IsNaN(tmp13))
            {
                tmp13 = 0;
                isUnreachable = true;
            }

            joints[1] = !elbow ? -tmp13 + tmp14 : tmp13 + tmp14;
        }
        else
        {
            var s2 = Sqrt(s2_2);
            var tmp6 = s2_2 + c2_2 - kappa_2;
            var tmp15 = Acos(tmp6 / (2.0 * s2 * c2));
            var tmp16 = Atan2(nx1 + 2.0 * a1, c.Z - c1);

            if (double.IsNaN(tmp15))
            {
                tmp15 = 0;
                isUnreachable = true;
            }

            joints[1] = !elbow ? tmp15 - tmp16 : -tmp15 - tmp16;
        }

        // Step 4: Solve Joint 3 (Elbow Angle)
        var tmp9 = 2.0 * c2 * Sqrt(kappa_2);
        var tmp10 = Atan2(a2, c3);

        if (!shoulder)
        {
            var tmp7 = s1_2 - c2_2 - kappa_2;
            var tmp11 = Acos(tmp7 / tmp9);

            if (double.IsNaN(tmp11))
            {
                tmp11 = 0;
                isUnreachable = true;
            }

            joints[2] = !elbow ? tmp11 - tmp10 : -tmp11 - tmp10;
        }
        else
        {
            var tmp8 = s2_2 - c2_2 - kappa_2;
            var tmp12 = Acos(tmp8 / tmp9);

            if (double.IsNaN(tmp12))
            {
                tmp12 = 0;
                isUnreachable = true;
            }

            joints[2] = !elbow ? -tmp12 - tmp10 : tmp12 - tmp10;
        }

        // Setup base orientation matrices for the wrist projection
        double sin = Sin(joints[0]);
        double cos = Cos(joints[0]);
        double s23 = Sin(joints[1] + joints[2]);
        double c23 = Cos(joints[1] + joints[2]);

        // Step 5: Solve Joint 5 (Wrist Pitch Angle)
        double m = t.M02 * s23 * cos + t.M12 * s23 * sin + t.M22 * c23;
        joints[4] = Atan2(Sqrt(1 - m * m), m);

        if (wrist)
            joints[4] = -joints[4];

        const double zero_threshold = 1.24e-2;

        // Step 6: Solve Joint 4 (Wrist Roll) and Joint 6 (Flange Spin)
        if (Abs(joints[4]) < zero_threshold)
        {
            // Handle Singularity State (Gimbal Lock)
            isSingularity = true;
            joints[3] = 0; // Lock J4 and dump entire remaining rotation onto J6

            Vector3d xe = new(t.M00, t.M10, t.M20);
            Vector3d col1 = new(-Sin(joints[0]), Cos(joints[0]), 0);
            Vector3d col2 = t.GetColumn3d(2);
            var col0 = Vector3d.CrossProduct(col1, col2);

            Transform rc = default;
            rc.SetRotation(
                col0.X, col0.Y, col0.Z,
                col1.X, col1.Y, col1.Z,
                col2.X, col2.Y, col2.Z
            );

            Vector3d xec = rc * xe;
            joints[5] = Atan2(xec[1], xec[0]);
        }
        else
        {
            // Normal Non-Singular Case
            var joints3_iy = t.M12 * cos - t.M02 * sin;
            var joints3_ix = t.M02 * c23 * cos + t.M12 * c23 * sin - t.M22 * s23;
            joints[3] = Atan2(joints3_iy, joints3_ix);

            var joints5_iy = t.M01 * s23 * cos + t.M11 * s23 * sin + t.M21 * c23;
            var joints5_ix = -t.M00 * s23 * cos - t.M10 * s23 * sin - t.M20 * c23;
            joints[5] = Atan2(joints5_iy, joints5_ix);
        }

        if (wrist)
        {
            joints[3] += PI;
            joints[5] -= PI;
        }

        // Step 7: Apply Phase Shifts and Orientation Remappings for the Yaskawa Base State
        // _start handles the right-angle calibration shifts we found in your 3DM frames
        // _signs accounts for inverted motor spin orientations relative to standard OPW math
        for (int i = 0; i < 6; i++)
        {
            joints[i] = _signs[i] * joints[i] + _start[i];

            if (joints[i] > PI) joints[i] -= 2 * PI;
            if (joints[i] < -PI) joints[i] += 2 * PI;

            if (double.IsNaN(joints[i]))
                joints[i] = 0;
        }

        if (isUnreachable)
            errors.Add($"Target out of reach.");

        if (isSingularity)
            errors.Add($"Target near singularity.");

        return joints;
    }
}
