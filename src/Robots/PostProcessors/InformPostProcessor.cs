
namespace Robots;

//TODO: Implement INFORM post processor
class InformPostProcessor : IPostProcessor
{
    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        // throw new NotImplementedException("INFORM post processor is not yet implemented.");
        PostInstance instance = new((SystemYaskawa)system, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemYaskawa _system;
        readonly Program _program;
        public List<List<List<string>>> Code { get; }

        private int p_variable;

        private int curConfig = -1;

        public PostInstance(SystemYaskawa system, Program program)
        {
            _system = system;
            _program = program;
            Code = [];

            PostProcessorUtil.RejectMultiRobot(program, _system, "Yaskawa");
            PostProcessorUtil.RejectExternalAxes(program, _system, "Yaskawa");

            for (int i = 0; i < _system.MechanicalGroups.Count; i++)
            {
                List<List<string>> groupCode = [MainModule(i)];

                for (int j = 0; j < program.MultiFileIndices.Count; j++)
                    groupCode.Add(SubModule(j, i));

                Code.Add(groupCode);
            }
        }

        List<string> MainModule(int group)
        {
            List<string> code = [];
            bool multiProgram = _program.MultiFileIndices.Count > 1;

            // Program Start
            code.Add("/JOB");
            code.Add($"//NAME {_program.Name}");
            code.Add("//POS");
            code.Add("/MAIN");
            code.Add("NOP");

            if (group == 0)
                PostProcessorUtil.AddInitCommands(code, _program);

            if (multiProgram)
            {
                for (int i = 0; i < _program.MultiFileIndices.Count; i++)
                    code.Add($"CALL JOB:{_program.Name}_{i:000}");
            }

            if (multiProgram)
            {
                code.Add("END");
                code.Add("");
            }

            return code;
        }

        List<string> SubModule(int file, int group)
        {
            bool multiProgram = _program.MultiFileIndices.Count > 1;
            var (start, end) = _program.GetTargetRange(file);

            List<string> code = [];

            if (multiProgram)
            {
                code.Add("/JOB");
                code.Add($"//NAME {_program.Name}_{file:000}");
                code.Add("NOP");
            }

            for (int j = start; j < end; j++)
            {
                var programTarget = _program.Targets[j].ProgramTargets[group];
                var target = programTarget.Target;
                string moveText;
                string curP = GetPVariable();
                // string zone = (target.Zone.IsFlyBy ? $"CNT{target.Zone.Distance}" : "FINE").NotNull("Zone name cannot be null.");

                if (programTarget.IsJointTarget)
                {
                    var jointTarget = (JointTarget)target;
                    var joints = jointTarget.Joints; // Assuming this array holds degrees. If radians, convert using * (180.0 / Math.PI)

                    // Force the register to a Pulse Type using P000 as a structural template
                    code.Add($"SET P{curP} P000");

                    // INFORM expects joint angles as integers representing 1/10000th of a degree
                    for (int axis = 0; axis < joints.Length; axis++)
                    {
                        int informValue = (int)Math.Round(joints[axis] * 10000.0);
                        code.Add($"SETE P{curP} ({axis + 1}) {informValue}");
                    }

                    int percentSpeed = GetAxisSpeed(programTarget, _system.MechanicalGroups[group].Robot.Joints);
                    moveText = $"MOVJ P{curP} VJ={Math.Clamp(percentSpeed, 1, 100)} T={(jointTarget.Tool.Number == null ? 0 : jointTarget.Tool.Number)}";
                }
                else
                {
                    var cartesian = (CartesianTarget)target;
                    var plane = cartesian.Plane;
                    var planeValues = _system.PlaneToNumbers(plane);

                    code.Add($"SET P{curP} X{planeValues[0]:0.000} Y{planeValues[1]:0.000} Z{planeValues[2]:0.000} Rx{planeValues[3]:0.000} Ry{planeValues[4]:0.000} Rz{planeValues[5]:0.000}");

                    RobotConfigurations configuration = programTarget.Kinematics.Configuration;
                    bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                    bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                    if (shoulder) elbow = !elbow;
                    bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                    int config = (shoulder ? 1 : 0) + (elbow ? 2 : 0) + (wrist ? 4 : 0);
                    if (config != curConfig)
                        code.Add($"SETE P{curP} (7) {config}");
                    curConfig = config;

                    switch (cartesian.Motion)
                    {
                        case Motions.Joint:
                            {
                                moveText = $"MOVJ P{curP} VJ={target.Speed.TranslationSpeed} T={(target.Tool.Number == null ? 0 : target.Tool.Number)}";

                                break;
                            }

                        case Motions.Linear:
                            {
                                moveText = $"MOVL P{curP} V={target.Speed.TranslationSpeed} T={target.Tool.Number}";

                                break;
                            }
                        default:
                            throw PostProcessorUtil.InvalidMotion(cartesian.Motion);
                    }
                }

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, true, TargetCommand);

                code.Add(moveText);

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, false, TargetCommand);

            }

            if (multiProgram)
                code.Add("RET");
            else
                code.Add("END");

            code.Add("");
            return code;
        }

        static string TargetCommand(string command) =>
            command;

        // TODO: Frames are not used.
        // TODO: Implement tool changes and use tool-specific variables in move commands.

        static int GetAxisSpeed(ProgramTarget programTarget, Joint[] joints)
        {
            double percentSpeed;
            var jointTarget = (JointTarget)programTarget.Target;

            if (programTarget.SystemTarget.DeltaTime > 0)
            {
                percentSpeed = Math.Round(programTarget.SystemTarget.MinTime / programTarget.SystemTarget.DeltaTime * 100.0, 0);
            }
            else
            {
                const double maxTranslationSpeed = 1000.0;
                double leadAxisSpeed = joints.Max(j => j.MaxSpeed);
                double percentage = Math.Min(jointTarget.Speed.TranslationSpeed / maxTranslationSpeed, 1);
                percentSpeed = Math.Round(percentage * leadAxisSpeed, 0);
            }

            return (int)percentSpeed;
        }

        private string GetPVariable()
        {
            return p_variable++.ToString("D3");
        }
    }
}
