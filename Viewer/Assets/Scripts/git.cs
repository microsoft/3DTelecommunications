using System;
using UnityEngine;
using System.Collections;

using System.Diagnostics;

public static class Git
{
    /* Properties ============================================================================================================= */

    /// <summary>
    /// Retrieves the build version from git based on the most recent matching tag and
    /// commit history. This returns the version as: {major.minor.build} where 'build'
    /// represents the nth commit after the tagged commit.
    /// Note: The initial 'v' and the commit hash code are removed.
    /// </summary>
    public static string BuildVersion
    {
        get
        {
            const string default_version = "0.0.0";

            try
            {
                string version = Run(@"describe --tags --long");

                if( version == null )
                {
                    return default_version;
                }

                int markerIndex = version.IndexOf( "_v");
                if( markerIndex < 0 )
                {
                    return default_version;
                }

                string szVersionPart = version.Substring( markerIndex + 2 );
                string [] szSplits = szVersionPart.Split( '.' );
                // any of the szSplits[x] can have non-alpha-numeric characters in them!
                for( int i = 0 ; i < szSplits.Length ; i++ )
                {
                    string s = szSplits[i];
                    int j;
                    for( j = 0 ; j < s.Length ; j++ )
                    {
                        char c = s[j];
                        if( c < '0' || c > '9' )
                        {
                            break;
                        }
                    }
                    if( j > 0 )
                    {
                        szSplits [i] = s.Substring( 0, j );
                    }
                    else
                    {
                        szSplits [i] = "0";
                    }
                }

                string[] szThreeSplits = new string[3];
                for( int i = 0 ; i < 3 ; i++ )
                {
                    if( i < szSplits.Length )
                    {
                        szThreeSplits [i] = szSplits [i];
                    }
                    else
                    {
                        szThreeSplits [i] = "0";
                    }
                }

                //====================================
                // replace patch # with local log time
                //====================================

                string szLatestCommitTime = Git.LatestCommitTime;
                szThreeSplits [2] = szLatestCommitTime;

                version = szThreeSplits [0] + "." + szThreeSplits [1] + "." + szThreeSplits [2];

                return version;
            }
            catch
            {
                return default_version;
            }
        }
    }

    public static string LatestCommitTime
    {
        get
        {
            try
            {
                string version = Run(@"log -1 --format=%ct");
                return version;
            }
            catch
            {
                return "0";
            }
        }
    }

    /// <summary>
    /// The currently active branch.
    /// </summary>
    public static string Branch => Run( @"rev-parse --abbrev-ref HEAD" );

    public static string HeadSha1 => Run( @"rev-parse HEAD" );

    /// <summary>
    /// Returns a listing of all uncommitted or untracked (added) files.
    /// </summary>
    public static string Status => Run( @"status --porcelain" );


    /* Methods ================================================================================================================ */

    /// <summary>
    /// Runs git.exe with the specified arguments and returns the output.
    /// </summary>
    public static string Run( string arguments )
    {
        using( var process = new System.Diagnostics.Process( ) )
        {
            var exitCode = process.Run(@"git", arguments, Application.dataPath,
                out var output, out var errors);
            if( exitCode == 0 )
            {
                return output;
            }
            else
            {
                throw new GitException( exitCode, errors );
            }
        }
    }
}

/// <summary>
/// GitException includes the error output from a Git.Run() command as well as the
/// ExitCode it returned.
/// </summary>
public class GitException : InvalidOperationException
{
    public GitException( int exitCode, string errors ) : base( errors ) =>
        this.ExitCode = exitCode;

    /// <summary>
    /// The exit code returned when running the Git command.
    /// </summary>
    public readonly int ExitCode;
}

