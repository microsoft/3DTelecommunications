using Assets.Scripts.Common;
using Assets.Scripts.Common.Extensions;
using Assets.Scripts.Viewer;
using Assets.Scripts.Viewer.Models;
using HoloportationDecoders;
using SharpConfig;
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;


public class SettingsManager : MonoBehaviour
{

    private static Color[] DEFAULT_DRAW_COLORS = new Color[] {
        Color.white,
        Color.red,
        Color.blue,
        Color.yellow,
        Color.green,

        // Bright pink
        new Color(1.0f, 0f, 0.5f)
    };

    private static Vector2 DEFAULT_PRESENTER_CAMERA_RESOLUTION = new Vector2(640, 480);

    private static readonly string uiSectionKey = "UI";
    private static readonly string uiTraditonalSectionKey = "UI/Traditional";
    private static readonly string uiHoloportationSectionKey = "UI/Holoportation";


    // Keys for settings so there is one source of truth for where each setting is stored within the config
    public SettingName CameraPanningSpeedKey = 
        new SettingName(uiSectionKey, "CameraPanningSpeed");
    public SettingName CameraRotationSpeedKey = 
        new SettingName(uiSectionKey, "CameraRotationSpeed");
    public SettingName CameraZoomSpeedKey = 
        new SettingName(uiSectionKey, "CameraZoomSpeed");
    public SettingName ZoomToolScaleFactorKey = 
        new SettingName(uiSectionKey, "ZoomToolScaleFactor");
    public SettingName SpotlightSizeKey =
        new SettingName(uiSectionKey, "SpotlightSize");
    public SettingName SpotlightMaskColorKey =
        new SettingName(uiSectionKey, "SpotlightMaskColor");
    public SettingName ZoomEnabledTraditional =
        new SettingName(uiTraditonalSectionKey, "ZoomEnabled");
    public SettingName ZoomEnabledHoloportation =
        new SettingName(uiHoloportationSectionKey, "ZoomEnabled");
    public SettingName PanEnabledTraditional =
        new SettingName(uiTraditonalSectionKey, "PanEnabled");
    public SettingName PanEnabledHoloportation =
        new SettingName(uiHoloportationSectionKey, "PanEnabled");
    public SettingName CameraCropsTraditional =
        new SettingName(uiTraditonalSectionKey, "CameraCrops");
    public SettingName CameraCropsHoloportation =
        new SettingName(uiHoloportationSectionKey, "CameraCrops");
    public SettingName DrawToolBrushColorsKey =
        new SettingName("DrawTool", "PaletteColors");
    public SettingName DrawToolBrushHardnessKey =
        new SettingName("DrawTool", "BrushHardness");
    public SettingName DrawToolBrushColorKey =
        new SettingName("DrawTool", "BrushColor");
    public SettingName DrawToolBrushOpacityKey =
        new SettingName("DrawTool", "BrushOpacity");
    public SettingName DrawToolBrushSizeKey =
        new SettingName("DrawTool", "BrushSize");
    public SettingName PresenterCameraNameSearchStringKey =
        new SettingName("Presenter", "CameraNameSearchString");
    public SettingName PresenterFallbackCameraIndexKey =
        new SettingName("Presenter", "FallbackCameraIndex");
    public SettingName PresenterCameraResolutionKey =
        new SettingName("Presenter", "CameraResolution");

    //threadsafe SINGLETON pattern
    protected static SettingsManager instance;

    /// <summary>
    /// Internal representation of the settings
    /// Don't use directly! All calls to this config should be instead routed through SettingsManager.
    /// Specifically SettingsManager.GetOrDefault has some additional logic
    /// </summary>
    private Configuration __config;

    [SerializeField]
    public string fileName = "Holoportation.cfg";
    
    [SerializeField]
    public string alternateFileName = "3DTelemedicine.cfg";

    public delegate void SettingsChangedEventHandler(SettingName setting, object value);


    public event SettingsChangedEventHandler OnChange;

    private string snapshotDirectory;

    // Explicit static constructor to tell C# compiler
    // not to mark type as beforefieldinit
    static SettingsManager()
    {
        Configuration.RegisterTypeStringConverter(
            new IntToRectMapTypeConverter()
        );
    }

    /// <summary>
    /// The singleton instance of this settings manager
    /// </summary>
    public static SettingsManager Instance
    {
        get
        {
            return instance;
        }
    }

    /// <summary>
    /// The offset of the front camera in 3d space
    /// </summary>
    public virtual Vector3 Front3DCameraOffset
    {
        get
        {
            return GetOrDefault(uiSectionKey, "Front3DCameraOffset", Vector3.zero);
        }
    }

    /// <summary>
    /// The offset of the back camera in 3d space
    /// </summary>
    public virtual Vector3 Back3DCameraOffset
    {
        get
        {
            return GetOrDefault(uiSectionKey, "Back3DCameraOffset", Vector3.zero);
        }
    }

    /// <summary>
    /// The offset of the left camera in 3d space
    /// </summary>
    public virtual Vector3 Left3DCameraOffset
    {
        get
        {
            return GetOrDefault(uiSectionKey, "Left3DCameraOffset", Vector3.zero);
        }
    }

    /// <summary>
    /// The offset of the right camera in 3d space
    /// </summary>
    public virtual Vector3 Right3DCameraOffset
    {
        get
        {
            return GetOrDefault(uiSectionKey, "Right3DCameraOffset", Vector3.zero);
        }
    }

    /// <summary>
    /// The offset of the top camera in 3d space
    /// </summary>
    public virtual Vector3 Top3DCameraOffset
    {
        get
        {
            return GetOrDefault(uiSectionKey, "Top3DCameraOffset", Vector3.zero);
        }
    }

    /// <summary>
    /// The folder to use for the color2d tutorial data
    /// </summary>
    public virtual string TutorialColor2DFolder
    {
        get
        {
            return GetOrDefault("Tutorial", "Color2DDataFolder", (string)null);
        }
    }

    /// <summary>
    /// The folder to use for the holoportation tutorial data
    /// </summary>
    public virtual string TutorialHoloportationFolder
    {
        get
        {
            return GetOrDefault("Tutorial", "HoloportDataFolder", (string)null);
        }
    }

    /// <summary>
    /// The number of available cameras
    /// </summary>
    public virtual int NumCameras
    {
        get
        {
            return GetOrDefault("DepthGeneration", "DepthCameraCount", 0);
        }
    }

    /// <summary>
    /// The default 2d camera to use
    /// </summary>
    public virtual int Default2DCamera
    {
        get
        {
            return GetOrDefault(
                uiSectionKey, 
                "Default2DCamera",
                0);
        }
    }

    /// <summary>
    /// The default mode the app should start in
    /// i.e. Holoportation or Traditional
    /// </summary>
    public virtual ViewerMode DefaultViewerMode
    {
        get
        {
            ViewerMode mode = ViewerMode.None;
            string modeStr = GetOrDefault(uiSectionKey, "DefaultMode", (string)null);
            if (!string.IsNullOrWhiteSpace(modeStr))
            {
                modeStr = modeStr.ToLower();
                if (modeStr.Contains("holo"))
                {
                    mode = ViewerMode.Holoportation;
                } else if (modeStr.Contains("traditional") || modeStr.Contains("basic"))
                {
                    mode = ViewerMode.Traditional;
                }
            }
            return mode;
        }
    }

    /// <summary>
    /// The default directory to use to save snapshots
    /// </summary>
    public virtual string SnapshotsDirectory
    {
        get
        {
            if (this.snapshotDirectory != null)
            {
                return Path.Combine(snapshotDirectory, "./Telemedicine/snapshots");
            }
       
            string defaultDirectory = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "./Telemedicine/snapshots");
            return GetOrDefault("Decoder", "SnapshotsDirectory", defaultDirectory);
        }
        set
        {
            this.snapshotDirectory = value;
        }
    }

    /// <summary>
    /// If the debug FPS meter should be shown
    /// </summary>
    public virtual bool ShowFPS
    {
        get
        {
            return GetOrDefault("Decoder", "ShowFPS", false);
        }
    }

    /// <summary>
    /// The spotlight size
    /// </summary>
    public virtual float SpotlightSize
    {
        get
        {
            return Mathf.Clamp(GetOrDefault(SpotlightSizeKey, 0.1f), 0.01f, 1f);
        }
    }

    /// <summary>
    /// The spotlight mask color
    /// </summary>
    public virtual Color SpotlightMaskColor
    {
        get
        {
            return GetOrDefault(SpotlightMaskColorKey, new Vector4(0.0f, 0.0f, 0.0f, 0.1f));
        }
    }

    /// <summary>
    /// The camera panning speed
    /// </summary>
    public virtual float CameraPanningSpeed
    {
        get
        {
            return GetOrDefault(CameraPanningSpeedKey, 0.5f);
        }
    }

    /// <summary>
    /// The camera rotation speed
    /// </summary>
    public virtual float CameraRotationSpeed
    {
        get
        {
            return GetOrDefault(CameraRotationSpeedKey, 0.5f);
        }
    }

    /// <summary>
    /// The camera zoom speed
    /// </summary>
    public virtual float CameraZoomSpeed
    {
        get
        {
            return GetOrDefault(CameraZoomSpeedKey, 0.5f);
        }
    }

    /// <summary>
    /// The color of the pan tool
    /// </summary>
    public virtual Color PanToolColor
    {
        get
        {
            return GetOrDefault(uiSectionKey, "PanToolColor", Color.white);
        }
    }

    /// <summary>
    /// The color of the orbit tool
    /// </summary>
    public virtual Color OrbitToolColor
    {
        get
        {
            return GetOrDefault(uiSectionKey, "OrbitToolColor", Color.white);
        }
    }

    /// <summary>
    /// The color of the zoom tool
    /// </summary>
    public virtual Color ZoomToolColor
    {
        get
        {
            return GetOrDefault(uiSectionKey, "ZoomToolColor", Color.white);
        }
    }

    /// <summary>
    /// The scaling factor the zoom tool should use when the user clicks
    /// </summary>
    public virtual float ZoomToolScaleFactor
    {
        get
        {
            return GetOrDefault(ZoomToolScaleFactorKey, 0.2f);
        }
    }

    /// <summary>
    /// If the low quality should be saved when creating a snapshot
    /// </summary>
    public virtual bool CaptureLQ3DModel
    {
        get
        {
            return GetOrDefault("Capture", "3DModelLowQuality", true);
        }
    }

    /// <summary>
    /// If the high quality should be saved when creating a snapshot
    /// </summary>
    public virtual bool CaptureHQ3DModel
    {
        get
        {
            return GetOrDefault("Capture", "3DModelHighQuality", true);
        }
    }

    /// <summary>
    /// If a screenshot should be saved when creating a snapshot
    /// </summary>
    public virtual bool CaptureScreenshot
    {
        get
        {
            return GetOrDefault("Capture", "Screenshot", true);
        }
    }

    /// <summary>
    /// If the current state (zoom/pan) of the views should should be saved when creating a snapshot
    /// </summary>
    public virtual bool CaptureViews
    {
        get
        {
            return GetOrDefault("Capture", "Views", true);
        }
    }

    /// <summary>
    /// If the raw frames directly from the pods should be saved when creating a snapshot
    /// </summary>
    public virtual bool CaptureAllRawFrames
    {
        get
        {
            return GetOrDefault("Capture", "AllRawFrames", true);
        }
    }

    /// <summary>
    /// If HQ mode should be toggled when the capture button is clicked
    /// </summary>
    public virtual bool ToggleHQOnCapture
    {
        get
        {
            return GetOrDefault("Capture", "ToggleHQ", true);
        }
    }

    /// <summary>
    /// The default frequency at which the 2d views should be updated
    /// </summary>
    public virtual int Default2DUpdateFPS
    {
        get
        {
            return GetOrDefault(uiSectionKey, "Default2DUpdateFPS", 1);
        }
    }
    
    /// <summary>
    /// If any zoom functionality is enabled
    /// </summary>
    public bool ZoomEnabled
    {
        get
        {
            return GetForCurrentAppMode(
                ZoomEnabledHoloportation,
                ZoomEnabledTraditional,
                true
            );
        }
    }

    /// <summary>
    /// If any pan functionality is enabled
    /// </summary>
    public bool PanEnabled
    {
        get
        {
            return GetForCurrentAppMode(
                PanEnabledHoloportation,
                PanEnabledTraditional,
                true
            );
        }
    }

    /// <summary>
    /// The set of croppings to apply to the cameras
    /// A mapping of cameraIdx: (<x>,<y>,<width>,<height>)
    /// </summary>
    public IReadOnlyDictionary<int, Rect> CameraCrops
    {
        get
        {
            return GetForCurrentAppMode(
                CameraCropsHoloportation,
                CameraCropsTraditional,
                null as IReadOnlyDictionary<int, Rect>
            );
        }
    }

    /// <summary>
    /// Should the room background grid be shown
    /// </summary>
    public bool RoomGrid
    {
        get
        {
            return GetOrDefault(uiSectionKey, "RoomGrid", false);
        }
    }

    /// <summary>
    /// Should the room support lighting
    /// </summary>
    public bool RoomShadows
    {
        get
        {
            return GetOrDefault(uiSectionKey, "RoomShadows", false);
        }
    }

    /// <summary>
    /// The color to use as the holoportation room background color
    /// </summary>
    public Color RoomBackgroundColor
    {
        get
        {
            return GetOrDefault(uiSectionKey, "RoomBackgroundColor", Color.white);
        }
    }

    /// <summary>
    /// The list of available drawing colors
    /// </summary>
    public virtual Color[] DrawToolColors
    {
        get
        {
            // https://dev.azure.com/msrp/PeabodyMain/_workitems/edit/9769
            //return GetOrDefaultArray(DrawToolBrushColorsKey, DEFAULT_DRAW_COLORS);
            return DEFAULT_DRAW_COLORS;
        }
    }

    /// <summary>
    /// The default brush hardness
    /// </summary>
    public virtual float DrawToolBrushHardness
    {
        get
        {
            return GetOrDefault(DrawToolBrushHardnessKey, 0.5f);
        }
        set
        {
            AddOrUpdate(DrawToolBrushHardnessKey, value);
            OnChange?.Invoke(DrawToolBrushHardnessKey, value);
        }
    }

    /// <summary>
    /// The default drawing tool color
    /// </summary>
    public virtual Color DrawToolBrushColor
    {
        get
        {
            return GetOrDefault(DrawToolBrushColorKey, DEFAULT_DRAW_COLORS[0]);
        }
        set
        {
            AddOrUpdate(DrawToolBrushColorKey, value);
            OnChange?.Invoke(DrawToolBrushColorKey, value);
        }
    }

    /// <summary>
    /// The default drawing tool opacity
    /// </summary>
    public virtual float DrawToolBrushOpacity
    {
        get
        {
            return GetOrDefault(DrawToolBrushOpacityKey, 1.0f);
        }
        set
        {
            AddOrUpdate(DrawToolBrushOpacityKey, value);
            OnChange?.Invoke(DrawToolBrushOpacityKey, value);
        }
    }

    /// <summary>
    /// The default drawing tool size
    /// </summary>
    public virtual float DrawToolBrushSize
    {
        get
        {
            return GetOrDefault(DrawToolBrushSizeKey, 0.01f);
        }
        set
        {
            AddOrUpdate(DrawToolBrushSizeKey, value);
            OnChange?.Invoke(DrawToolBrushSizeKey, value);
        }
    }

    /// <summary>
    /// The name of the camera to search for when loading presenter mode
    /// </summary>
    public virtual string PresenterCameraNameSearchString
    {
        get
        {
            return GetOrDefault(PresenterCameraNameSearchStringKey, "NVIDIA Broadcast");
        }
    }

    /// <summary>
    /// The index of the camera to use if the camera cannot be found by _PresenterCameraNameSearchString_
    /// </summary>
    public virtual int PresenterFallbackCameraIndex
    {
        get
        {
            return GetOrDefault(PresenterFallbackCameraIndexKey, 0);
        }
    }

    /// <summary>
    /// The resolution to use for the presenter camera
    /// </summary>
    public virtual Vector2 PresenterCameraResolution
    {
        get
        {
            // The use of <Vector3> here is intentional, it forces the SharpConfig to use our Vector3 parser
            // rather than needing to create another parser.
            return GetOrDefault<Vector3>(PresenterCameraResolutionKey, DEFAULT_PRESENTER_CAMERA_RESOLUTION);
        }
    }

    // Start is called before the first frame update
    void OnEnable()
    {
        if (instance == null)
        {
            instance = this;

            string envConfigPath = Environment.GetEnvironmentVariable("3DTELEMEDICINE_DIR");
            envConfigPath = string.IsNullOrWhiteSpace(envConfigPath) ? "C:\\3DTelemedicine" : envConfigPath;

            __config = ConfigurationLoader.LoadConfiguration(
                Path.Combine(Application.streamingAssetsPath, fileName),
                Path.Combine(Application.streamingAssetsPath, alternateFileName),
                Path.Combine(Application.dataPath, fileName),
                Path.Combine(Application.dataPath, alternateFileName),
                Path.Combine(Path.GetDirectoryName("."), fileName),
                Path.Combine(Path.GetDirectoryName("."), alternateFileName),
                Path.Combine(envConfigPath, fileName),
                Path.Combine(envConfigPath, alternateFileName),
                GetUserPreferencesPath()
            );

            // Clamp the preference values to their new range
            AddOrUpdate(CameraPanningSpeedKey, Mathf.Clamp01(CameraPanningSpeed));
            AddOrUpdate(CameraRotationSpeedKey, Mathf.Clamp01(CameraRotationSpeed));
            AddOrUpdate(CameraZoomSpeedKey, Mathf.Clamp01(CameraZoomSpeed));
            AddOrUpdate(ZoomToolScaleFactorKey, Mathf.Clamp01(ZoomToolScaleFactor));
            AddOrUpdate(SpotlightSizeKey, Mathf.Clamp(SpotlightSize, 0.01f, 1.0f));
        }
    }

    /// <summary>
    /// Saves the user preferences
    /// </summary>
    public void SaveUserPreferences()
    {
        Configuration userPref = new Configuration();
        userPref.AddOrUpdate(CameraPanningSpeedKey, CameraPanningSpeed);
        userPref.AddOrUpdate(CameraRotationSpeedKey, CameraRotationSpeed);
        userPref.AddOrUpdate(CameraZoomSpeedKey, CameraZoomSpeed);
        userPref.AddOrUpdate(ZoomToolScaleFactorKey, ZoomToolScaleFactor);
        userPref.AddOrUpdate(SpotlightSizeKey, SpotlightSize);
        userPref.AddOrUpdate(SpotlightMaskColorKey, SpotlightMaskColor);
        userPref.SaveToFile(GetUserPreferencesPath());
    }
    /// <summary>
    /// Adds or updates config value and add event emitter for setting
    /// </summary>
    public void AddOrUpdate(SettingName setting, object value)
    {
        __config.AddOrUpdate(setting.Section, setting.Setting, value);
        OnChange?.Invoke(setting, value);
    }

    /// <summary>
    /// Gets default setting value or defaults to given value
    /// </summary>
    public T GetOrDefault<T>(SettingName setting, T value)
    {
        return GetOrDefault(setting.Section, setting.Setting, value);
    }

    /// <summary>
    /// Gets default setting value or defaults to given value
    /// </summary>
    public T GetOrDefault<T>(string section, string setting, T value)
    {
        // Prefer a "*/Viewer" prefixed setting first
        /*
         i.e.

         [Capture/Viewer]     >   [Capture]
         Screenshot = true        Screenshot = true

        */
        return __config.GetOrDefault($"{section}/Viewer", setting, __config.GetOrDefault(section, setting, value));
    }
    
    /// <summary>
    /// Converts to IDecoderSettings
    /// </summary>
    /// <returns></returns>
    public IDecoderSettings ToDecoderSettings()
    {
        return HoloportationDecoders.Settings.FromSharpConfig(__config);
    }

    /// <summary>
    /// Saves out the current settings configuration out to disk
    /// </summary>
    /// <param name="toSave">The configuration to save</param>
    /// <param name="outFile">The path to the out file</param>
    public void SaveConfigToDisk(SharpConfig.Configuration toSave, string outFile)
    {
        toSave.SaveToFile(outFile);
    }

    /// <summary>
    /// Gets the path to the user preferences
    /// </summary>
    /// <returns></returns>
    private static string GetUserPreferencesPath()
    {
        return Path.Combine(Application.persistentDataPath, "user.prefs");
    }

    /// <summary>
    /// Gets the correct setting for the current application mode
    /// </summary>
    /// <typeparam name="T">The type of setting</typeparam>
    /// <param name="holoportation">The key to use for "Holoportation" mode</param>
    /// <param name="traditional">The key to use for "Traditional" mode</param>
    /// <param name="defaultValue">The default value to use for the settings</param>
    /// <returns>The setting value</returns>
    private static T GetForCurrentAppMode<T>(SettingName holoportation, SettingName traditional, T defaultValue)
    {
        ViewerMode mode = ViewerManager.Current().Store.GetState().AppMode.Value;
        SettingName toUse = mode == ViewerMode.Holoportation ? holoportation : traditional;
        return Instance.GetOrDefault(toUse, defaultValue);
    }
}