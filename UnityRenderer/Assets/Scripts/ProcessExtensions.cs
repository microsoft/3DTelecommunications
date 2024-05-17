using System.Diagnostics;
using System.Text;

public static class ProcessExtensions
{
    /* Properties ============================================================================================================= */

    /* Methods ================================================================================================================ */

    /// <summary>
    /// Runs the specified process and waits for it to exit. Its output and errors are
    /// returned as well as the exit code from the process.
    /// See: https://stackoverflow.com/questions/4291912/process-start-how-to-get-the-output
    /// Note that if any deadlocks occur, read the above thread (cubrman's response).
    /// </summary>
    public static int Run( this Process process, string application,
        string arguments, string workingDirectory, out string output,
        out string errors )
    {
        process.StartInfo = new ProcessStartInfo
        {
            CreateNoWindow = true,
            UseShellExecute = false,
            RedirectStandardError = true,
            RedirectStandardOutput = true,
            FileName = application,
            Arguments = arguments,
            WorkingDirectory = workingDirectory
        };

        // Use the following event to read both output and errors output.
        var outputBuilder = new StringBuilder();
        var errorsBuilder = new StringBuilder();
        process.OutputDataReceived += ( _, args ) => outputBuilder.AppendLine( args.Data );
        process.ErrorDataReceived += ( _, args ) => errorsBuilder.AppendLine( args.Data );

        // Start the process and wait for it to exit.
        process.Start( );
        process.BeginOutputReadLine( );
        process.BeginErrorReadLine( );
        process.WaitForExit( );

        output = outputBuilder.ToString( ).TrimEnd( );
        errors = errorsBuilder.ToString( ).TrimEnd( );
        return process.ExitCode;
    }
}
