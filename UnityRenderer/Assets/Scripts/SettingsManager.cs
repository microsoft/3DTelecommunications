using SharpConfig;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
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

    /// <summary>
    /// Gets the number of depth cameras used in the system
    /// </summary>
    public int NumPods
    {
        get
        {
            return GetValueWithDefault("DepthGeneration", "DepthCameraCount", 10);
        }
    }

    /// <summary>
    /// Gets the width of the color images produced by fusion
    /// </summary>
    public int ColorImageWidth 
    {
        get 
        {
            return GetValueWithDefault("ColorProcessing", "ColorImageWidth", 1920);
        }
    }

    /// <summary>
    /// Gets the height of the color images produced by fusion
    /// </summary>
    public int ColorImageHeight
    {
        get 
        {
            return GetValueWithDefault("ColorProcessing", "ColorImageHeight", 1080);
        }
    }

    /// <summary>
    /// Gets the number of bytes per pixel of the color images produced by fusion
    /// </summary>
    public int ColorPixelBytes
    {
        get 
        {
            return GetValueWithDefault("ColorProcessing", "ColorPixelBytes", 3);
        }
    }
    
    /// <summary>
    /// Gets the audio sample rate of the cameras
    /// </summary>
    public int AudioSampleRate
    {
        get 
        {
            return GetValueWithDefault("Audio", "AudioSampleRate", 48000);
        }
    }
    
    /// <summary>
    /// Gets the sample size of the cameras
    /// </summary>
    public int AudioSampleSize
    {
        get 
        {
            return GetValueWithDefault("Audio", "AudioSampleSize", 2);
        }
    }
    
    /// <summary>
    /// Gets the expected index count of the meshes from fusion
    /// </summary>
    public int ExpectedIndexCountHQ
    {
        get 
        {
            return GetValueWithDefault("ModelToTexture", "ExpectedIndexCountHQ", 4_000_000);
        }
    }

    /// <summary>
    /// Gets the expected vertex count of the meshes from fusion
    /// </summary>
    public int ExpectedVertexCountHQ
    {
        get 
        {
            return GetValueWithDefault("ModelToTexture", "ExpectedVertexCountHQ", 700_000);
        }
    }

    /// <summary>
    /// Gets the size of the textures used to texture the mesh
    /// </summary>
    
    public int TextureDimensionHQ
    {
        get 
        {
            return GetValueWithDefault("ModelToTexture", "TextureDimensionHQ", 4096);
        }
    }

    /// <summary>
    /// Gets the per triangle size of the textures used to texture the mesh
    /// </summary>
    public int PerTriangleTextureDimensionHQ
    {
        get 
        {
            return GetValueWithDefault("ModelToTexture", "PerTriangleTextureDimensionHQ", 32);
        }
    }

    /// <summary>
    /// Gets the target FPS for reading data from fusion
    /// </summary>
    public float FusionNetworkTargetFPS
    {
        get
        {
            return GetValueWithDefault("Renderer", "NetworkTargetFPS", 15.0f);
        }
    }

    /// <summary>
    /// Gets the number of data structs to use when processing networking events
    /// </summary>
    public int FusionNetworkProcessingPoolSize
    {
        get
        {
            return GetValueWithDefault("Renderer", "NetworkProcessingPoolSize", 6);
        }
    }

    /// <summary>
    /// Gets the IP of the fusion machine
    /// </summary>
    public string FusionIP
    {
        get
        {
            return GetValueWithDefault("Network", "FusionIPAddress", "127.0.0.1");
        }
    }

    /// <summary>
    /// Gets the port of the fusion machine
    /// </summary>
    public string FusionPort
    {
        get
        {
            return GetValueWithDefault("Ports", "DataStreamPort", "14900");
        }
    }

    /// <summary>
    /// If true, fusion frames are read from disk
    /// </summary>
    public bool ReadFusionFramesFromDisk
    {
        get
        {
            return GetValueWithDefault("Renderer", "ReadFusionFramesFromDisk", false);
        }
    }

    /// <summary>
    /// The intput folder used for fusion frames
    /// </summary>
    public string ReadFusionFramesFromDiskFolder
    {
        get
        {
            return GetValueWithDefault("Renderer", "ReadFusionFramesFromDiskFolder", $"{Directory.GetCurrentDirectory()}/fusionFrames");
        }
    }

    /// <summary>
    /// If true, the fusion frames are written to disk
    /// </summary>
    public bool WriteFusionFramesToDisk
    {
        get
        {
            return GetValueWithDefault("Renderer", "WriteFusionFramesToDisk", false);
        }
    }

    /// <summary>
    /// The output folder used for fusion frames
    /// </summary>
    public string WriteFusionFramesToDiskFolder
    {
        get
        {
            return GetValueWithDefault("Renderer", "WriteFusionFramesToDiskFolder", $"{Directory.GetCurrentDirectory()}/fusionFrames");
        }
    }

    /// <summary>
    /// Gets a setting by section name and setting name width a default of _defaultValue_
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="sectionName"></param>
    /// <param name="settingName"></param>
    /// <param name="defaultValue"></param>
    /// <param name="outputWarning"></param>
    /// <returns></returns>
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
                Debug.Log(String.Format("Warning! Config [{0}][{1}] Missing", sectionName, settingName));
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
                Debug.Log(String.Format("Warning! Config [{0}][{1}] Missing", sectionName, settingName));
            }
            return defaultValueArray;
        }
    }

    public SharpConfig.Configuration config;
    public string fileName = "3DTelemedicine.cfg";
    public string configFileLocation = "";

    // Start is called before the first frame update
    void Awake()
    {
        if (instance == null)
        {
            instance = this;
        }

        config = new SharpConfig.Configuration();
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
        if (!configurationLoaded)
        {
            Debug.Log("Could not find a config file.  Loading default config.");
            configurationLoaded = LoadConfigFromDisk(Application.dataPath + Path.DirectorySeparatorChar + fileName + ".default");
            if (configurationLoaded)
            {
                Debug.Log("[settings]Copying default config file to " + configFileLocation);
                try
                {
                    File.Copy(Application.dataPath + Path.DirectorySeparatorChar + fileName + ".default", configFileLocation);
                }
                catch (Exception e)
                {
                    Debug.Log("[settings]Sorry.  Couldn't copy the default config to " + configFileLocation + ": " + e.Message);
                }
            }
        }
        Debug.Log($"[SettingsManager] Config file loaded from {configFileLocation}");
    }


    public SharpConfig.Configuration GetConfigCopy()
    {
        //deep copy, so just reload again
        SharpConfig.Configuration newConfig = SharpConfig.Configuration.LoadFromFile(configFileLocation);
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
            config = SharpConfig.Configuration.LoadFromFile(path);

            if (config != null)
            {
                Debug.Log("[settings]Config file loaded successfully: " + path);
                return true;
            }
            return false;
        }
        catch (Exception e)
        {
            Debug.LogError("[settings]Failed to load Config file:  " + path + ". Error: " + e.Message);
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

    public int Verbosity()
    {
        return GetValueWithDefault<int>("Renderer", "Verbosity", 0);
    }
}
