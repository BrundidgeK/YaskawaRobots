using Rhino.Geometry;
using System.Text;

namespace Robots;

public class SystemYaskawa : IndustrialSystem
{

    internal SystemYaskawa(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups)
    {
    }

    public override Manufacturers Manufacturer => Manufacturers.Yaskawa;

    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYXDegreesToPlane(numbers);

    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYXDegrees(plane);

    protected override IPostProcessor GetDefaultPostprocessor() => new InformPostProcessor();

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException("Program code was not generated.");

        // INFORM scripts use the .JBI (Job) file format encoded in standard ASCII/ANSI
        const string extension = "JBI";
        var encoding = Encoding.ASCII;

        // Create the main output directory for this program
        string programFolder = Path.Combine(folder, program.Name);
        _ = Directory.CreateDirectory(programFolder);

        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < program.Code.Count; i++)
        {
            string group = MechanicalGroups[i].Name;

            // Save the main Job file
            {
                // Format: ProgramName_GroupName.JBI
                string fileName = $"{program.Name}_{group}.{extension}";
                string file = Path.Combine(programFolder, fileName);

                var code = multiProgram ? program.Code[i][0] : program.Code[i][0].Concat(program.Code[i][1]);
                var joinedCode = string.Join("\r\n", code);

                File.WriteAllText(file, joinedCode, encoding);
            }

            // Save any split sub-jobs if the program exceeds single file point-limits
            if (multiProgram)
            {
                for (int j = 1; j < program.Code[i].Count; j++)
                {
                    int index = j - 1;
                    // Format: ProgramName_GroupName_000.JBI
                    string fileName = $"{program.Name}_{group}_{index:000}.{extension}";
                    string file = Path.Combine(programFolder, fileName);

                    var joinedCode = string.Join("\r\n", program.Code[i][j]);
                    File.WriteAllText(file, joinedCode, encoding);
                }
            }
        }
    }
}