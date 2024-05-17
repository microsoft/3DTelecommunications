using SharpConfig;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices.WindowsRuntime;
using UnityEngine;

public class SettingsManager : MonoBehaviour
{

    //threadsafe SINGLETON pattern
    protected static SettingsManager instance;

    // Explicit static constructor to tell C# compiler
    // not to mark type as beforefieldinit
    static SettingsManager()
    {
    }

    private SettingsManager()
    {
    }

    public static SettingsManager Instance
    {
        get
        {
            return instance;
        }
    }

    public T GetValueWithDefault<T>(string sectionName, string settingName , T defaultValue, bool outputWarning = false)
    {
        if (config.Contains(sectionName, settingName))
        {
            return config[sectionName][settingName].GetValue<T>();
        }
        else
        {
            if(outputWarning)
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
    void Awake()
    {
        if (instance == null)
        {
            instance = this;
        }

        config = new Configuration();

        bool configurationLoaded = false;
        // Try to load the global configuration file
        if (Environment.GetEnvironmentVariable("3DTelemedicine_dir") != null)
        {
            configFileLocation = Environment.GetEnvironmentVariable("3DTELEMEDICINE_DIR") + Path.DirectorySeparatorChar + fileName;
            configurationLoaded = LoadConfigFromDisk(configFileLocation);          
        }
        // If that failed, try to load the file from within the application data path (old config method)
        if (!configurationLoaded)
        {
            configFileLocation = Application.dataPath + Path.DirectorySeparatorChar + fileName;
            configurationLoaded = LoadConfigFromDisk(configFileLocation);
        }
        // If that failed, load the default config file and save it to the global location
        if(!configurationLoaded)
        {
            OutputHelper.OutputLog("Could not find a config file.  Loading default config.");
            configurationLoaded = LoadConfigFromDisk(Application.dataPath + Path.DirectorySeparatorChar + fileName + ".default");
            if (configurationLoaded)
            {
                OutputHelper.OutputLog("[settings]Copying default config file to " + configFileLocation);
                try
                {
                    File.Copy(Application.dataPath + Path.DirectorySeparatorChar + fileName + ".default", configFileLocation);
                }
                catch (Exception e)
                {
                    OutputHelper.OutputLog("[settings]Sorry.  Couldn't copy the default config to " + configFileLocation + ": " + e.Message);
                }
            }
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
        if(String.IsNullOrEmpty(path))
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

    // Update is called once per frame
    void Update()
    {
        
    }
}
