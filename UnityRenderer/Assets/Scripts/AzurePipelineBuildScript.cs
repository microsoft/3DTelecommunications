#if UNITY_EDITOR
    using UnityEditor;
    using UnityEditor.Build;
    using UnityEditor.Build.Reporting;
    using UnityEngine;

// this is called from the command line so that Azure pipelines only builds the scene we actually care about there
public class AzurePipelineBuildScript : IPreprocessBuildWithReport
{
    public int callbackOrder { get { return 0; } }

    public static void BuildRendererMainScene()
    {
        // Put any other scenes you want built in the pipeline into this array
        string[] scenes = new string[] { "Assets/Scenes/MainScene_TextureGenerator.unity" };
        BuildReport report = BuildPipeline.BuildPlayer(scenes, "Build/HoloPortRenderer.exe", BuildTarget.StandaloneWindows64, BuildOptions.None);
        BuildSummary summary = report.summary;

        if (summary.result == BuildResult.Succeeded)
        {
            Debug.Log("Build succeeded: " + summary.totalSize + " bytes");
            foreach(var file in report.files)
            {
                Debug.Log("Created output file: " + file.path);
            }
        }

        if (summary.result == BuildResult.Failed)
        {
            Debug.Log("!!Build failed!!");
        }
        EditorApplication.Exit(0);
    }
    public void OnPreprocessBuild( BuildReport report )
    {
        SetBundleVersionNumber( );
    }

    void SetBundleVersionNumber( )
    {
        // call the powershell script first to create the config file
        System.Diagnostics.ProcessStartInfo startInfo = new System.Diagnostics.ProcessStartInfo();
        startInfo.RedirectStandardOutput = true;
        startInfo.UseShellExecute = false;
        startInfo.FileName = "powershell.exe";
        startInfo.Arguments = "-ExecutionPolicy Bypass ../Scripts/CreateIncludeFileFromGitVersion.ps1 ./Build/";
        System.Diagnostics.Process p = System.Diagnostics.Process.Start(startInfo);
        string q = "";
        while (!p.HasExited)
        {
            q += p.StandardOutput.ReadToEnd();
        }
        Debug.Log("Version script finished: ");
        Debug.Log(q);
    }
}

#endif
