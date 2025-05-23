// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Net.NetworkInformation;
using System.Runtime.CompilerServices;
using System.ServiceModel;
using NLog;

namespace ControlPanel
{
    public class OutputHelper
    {
        private static Logger logger = LogManager.GetCurrentClassLogger();
        private static ControlPanel controlPanel;
        
        public static void SetControlPanel(ControlPanel controlPanel)
        {
            OutputHelper.controlPanel = controlPanel;
        }

        public static void SetLogFilename(string filename)
        {
            string appDataPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "AppData", "LocalLow");
            string logDirectory = Path.Combine(appDataPath, "Microsoft Research", "Control Panel");
            Directory.CreateDirectory(logDirectory); // Ensure the directory exists
            string logFilePath = Path.Combine(logDirectory, filename);
            string oldLogFilePath = Path.Combine(logDirectory, filename + ".old");
            // Rename the file if it exists to .old
            if (File.Exists(logFilePath))
            {
                if (File.Exists(oldLogFilePath))
                {
                    File.Delete(oldLogFilePath);
                }
                File.Move(logFilePath, logFilePath + ".old");
            }
            // Set the log file path variable
            LogManager.Configuration.Variables["logFilePath"] = logFilePath;
            LogManager.ReconfigExistingLoggers();
            OutputHelper.OutputLog($"Log file path set to {logFilePath}", Verbosity.Warning);
        }
        
        // Could be called before control panel exists, or before filename was passed in, so just set a default filename and get started
        public static void Init()
        {
            LogManager.LoadConfiguration("nlog.config");
            OutputHelper.SetLogFilename("Output.Log");
        }
        public static void OutputLog(string message, Verbosity verbosity = Verbosity.Warning)
        {
            if (SettingsManager.Instance == null || SettingsManager.Instance.Verbosity() >= (int)verbosity)
            {
                DateTime now = DateTime.Now;
                switch (verbosity)
                {
                    case Verbosity.Fatal:
                        logger.Fatal("[" + now.ToString() + "]:" + message);
                        break;
                    case Verbosity.Error:
                        logger.Error("[" + now.ToString() + "]:" + message);
                        break;
                    case Verbosity.Warning:
                        logger.Warn("[" + now.ToString() + "]:" + message);
                        break;
                    case Verbosity.Info:
                        logger.Info("[" + now.ToString() + "]:" + message);
                        break;
                    case Verbosity.Debug:
                        logger.Debug("[" + now.ToString() + "]:" + message);
                        break;
                    case Verbosity.Trace:
                        logger.Trace("[" + now.ToString() + "]:" + message);
                        break;
                }
                if (OutputHelper.controlPanel != null)
                {
                    // check if controlPanel window handle has been created
                    if (OutputHelper.controlPanel.IsHandleCreated)
                    {
                        OutputHelper.controlPanel.Invoke((MethodInvoker)delegate
                        {
                            OutputHelper.controlPanel.UpdateStatusText("[" + now.ToString() + "]:" + message);
                            OutputHelper.controlPanel.UpdateLogText("[" + now.ToString() + "]:" + message);
                        });
                    }
                }
            }
        }
        public enum Verbosity
        {
            Fatal = 0,
            Error,
            Warning,
            Info,
            Debug,
            Trace
        }

    }

}