// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
using SharpConfig;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;

namespace ControlPanel
{
    public class ConfigItem
    {
        public string? Section { get; set; }
        public string? Setting { get; set; }
        public string? Value { get; set; }
    }

    public class SettingsManager
    {

        //threadsafe SINGLETON pattern
        protected static SettingsManager instance;

        public static SettingsManager Instance
        {
            get
            {
                return instance;
            }
        }

        public T GetValueWithDefault<T>(string sectionName, string settingName, T defaultValue, bool outputWarning = false)
        {
            if (config.Contains(sectionName, settingName))
            {
                return config[sectionName][settingName].GetValue<T>();
            }
            else
            {
                if (outputWarning)
                {
                    OutputHelper.OutputLog(String.Format("Warning! Config [{0}][{1}] Missing", sectionName, settingName), OutputHelper.Verbosity.Warning);
                }
                return defaultValue;
            }
        }

        public T[] GetValueArrayWithDefault<T>(string sectionName, string settingName, T[] defaultValueArray, bool outputWarning = true)
        {
            if (config.Contains(sectionName, settingName))
            {
                return config[sectionName][settingName].GetValueArray<T>();
            }
            else
            {
                if (outputWarning)
                {
                    OutputHelper.OutputLog(String.Format("Warning! Config [{0}][{1}] Missing", sectionName, settingName), OutputHelper.Verbosity.Warning);
                }
                return defaultValueArray;
            }
        }
        public int Verbosity()
        {
            return GetValueWithDefault("Debug", "Verbosity", 0, false);
        }

        public Configuration config;
        public const string fileName = "3DTelemedicine.cfg";
        public string configFileLocation = "";
        // Awake is called before the first frame update
        public static void Init()
        {
            if (instance == null)
            {
                instance = new SettingsManager();
            }

            instance.config = new Configuration();

            bool configurationLoaded = false;
            // Try to load the global configuration file
            if (Environment.GetEnvironmentVariable("3DTelemedicine_dir") != null)
            {
                instance.configFileLocation = Environment.GetEnvironmentVariable("3DTELEMEDICINE_DIR") + Path.DirectorySeparatorChar + fileName;
                configurationLoaded = instance.LoadConfigFromDisk(instance.configFileLocation);
            }
            // check if C:\3DTelemedicine\3DTelemedicine.cfg exists
            if (!configurationLoaded)
            {
                instance.configFileLocation = "C:\\3DTelemedicine" + Path.DirectorySeparatorChar + fileName;
                configurationLoaded = instance.LoadConfigFromDisk(instance.configFileLocation);
            }
            // If that failed, load the default config file and save it to the global location
            if (!configurationLoaded)
            {
                OutputHelper.OutputLog("[settings]Sorry.  I couldn't find the config file.", OutputHelper.Verbosity.Fatal);
            }
        }


        public SharpConfig.Configuration GetConfigCopy()
        {
            //deep copy, so just reload again
            SharpConfig.Configuration newConfig = Configuration.LoadFromFile(configFileLocation);
            return newConfig;
        }

        public bool LoadConfigFromDisk(string path = null)
        {
            if (String.IsNullOrEmpty(path))
            {
                path = configFileLocation;
            }
            try
            {
                config = Configuration.LoadFromFile(path);

                if (config != null)
                {
                    OutputHelper.OutputLog("[settings]Config file loaded successfully: " + path);
                    return true;
                }
                return false;
            }
            catch (Exception e)
            {
                OutputHelper.OutputLog("[settings]Failed to load Config file:  " + path + " " + e.Message);
                return false;
            }
        }
        public void SaveConfigToDisk(SharpConfig.Configuration con)
        {
            con.SaveToFile(configFileLocation);
        }

    }

}