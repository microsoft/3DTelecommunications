using System;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
#endif
using UnityEngine;

#if UNITY_EDITOR
public class EditorOnlyClass : IPreprocessBuildWithReport
{
    public int callbackOrder { get { return 0; } }

    public void OnPreprocessBuild( BuildReport report )
    {
        Debug.Log( "OnPreprocessBuild for target " + report.summary.platform + " at path " + report.summary.outputPath );

        SetBundleVersionNumber( );
    }

    void SetBundleVersionNumber( )
    {
        string szBuildVersion = Git.BuildVersion;
        Debug.Log( "SetBundleVersionNumber, TM_BUILD_VERSION = " + szBuildVersion );

        string szBuildSha1 = Git.HeadSha1;
        Debug.Log( "SetBundleVersionNumber, TM_BUILD_SHA1 = " + szBuildSha1 );

        PlayerSettings.bundleVersion = szBuildVersion;
    }
}
#endif
