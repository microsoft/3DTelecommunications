using System.Collections;
using System.Collections.Generic;
using System.Text.Json;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class SettingsMain : TabController
{
    [Header("UI hookup between Data and Display")]
    public GameObject SectionsPanel;
    public GameObject SectionsPrefab;

    public GameObject SettingsPanel;
    public GameObject SettingsEntryPrefab;

    public GameObject MessageCanvas;

    private SharpConfig.Configuration configCopy;
    private SharpConfig.Section currSection; // currently selected section
    //instanced after initializing them
    private List<GameObject> sectionsList;
    private List<GameObject> settingsList;

    public Text versionText;

    private PeabodyNetworkingLibrary.VersionData versionData;

    // Start is called before the first frame update
    void Start()
    {
        settingsList = new List<GameObject>();
        PopulateSectionsFromConfig();
        versionData = PeabodyNetworkingLibrary.Utility.ReadVersionDataFromConfig(System.AppDomain.CurrentDomain.BaseDirectory + "\\3dtm_version.config");
        Debug.Log($"Control Panel Version {versionData.Major}.{versionData.Minor}.{versionData.Patch}-{versionData.BranchName}.{versionData.Commits} ({versionData.Description}) ({versionData.Sha1})");

        versionText.text = versionData.Description;
    }

    public void PopulateSectionsFromConfig()
    {
        if(sectionsList == null)
        {
            sectionsList = new List<GameObject>();
        }
        else
        {
            sectionsList.Clear();
        }
        
        SharpConfig.Configuration myConfig = SettingsManager.Instance.config;
        configCopy = SettingsManager.Instance.GetConfigCopy();
        foreach (SharpConfig.Section section in myConfig)
        {
            OutputHelper.OutputLog("Creating Section: " + section.Name);
            GameObject currSection = Instantiate(SectionsPrefab);
            currSection.name = section.Name;       
            currSection.transform.SetParent(SectionsPanel.transform, false);

            currSection.GetComponentInChildren<Text>().text = section.Name;
            currSection.GetComponentInChildren<Button>().onClick.AddListener(()=> { SwitchToSection(section.Name);  });
            currSection.SetActive(true);

            sectionsList.Add(currSection);
        }
        SectionsPrefab.SetActive(false);
    }

    public void SwitchToSection(string sectionName)
    {
        currSection = SettingsManager.Instance.config[sectionName];

        SettingsPanel.transform.Find("CurrSectionSettingTitle").GetComponent<Text>().text = sectionName.ToUpper() + " SETTINGS";
        int index = 0;
        foreach (SharpConfig.Setting setting in currSection)
        {
            GameObject currSetting = null;
            if (settingsList.Count <= index)
            {
                currSetting = Instantiate(SettingsEntryPrefab);

                currSetting.name = "Setting " + index;
                currSetting.transform.SetParent(SettingsPanel.transform);
                settingsList.Add(currSetting);
            }
            else
            {
                currSetting = settingsList[index];
            }
            currSetting.SetActive(true);
            //update values in the entry
            currSetting.transform.Find("SettingsTitleText").GetComponent<Text>().text = setting.Name;
            InputField entryInputField = currSetting.transform.Find("InputField").GetComponent<InputField>();
            if (setting.IsArray)
            {
                string[] strArray = setting.StringValueArray;
                string concat = string.Join(",", setting.StringValueArray);
                entryInputField.GetComponent<InputField>().text = "{" + concat + "}";
            }
            else
            {
                entryInputField.text = setting.StringValue;
            }
            entryInputField.interactable = true;
            entryInputField.onEndEdit.RemoveAllListeners();
            entryInputField.onEndEdit.AddListener((string fieldEntry) => 
            {
                configCopy[currSection.Name][setting.Name].SetValue(fieldEntry);                
            });
            
            index++;
        }

        for(int i = index; i < settingsList.Count; ++i)
        {
            settingsList[i].SetActive(false);
        }
        SettingsEntryPrefab.SetActive(false);
    }
    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnButton_SaveData()
    {
        SettingsManager.Instance.SaveConfigToDisk(configCopy);
        //if it's pipeline related, everything needs to reset (daemon/statusbot...etc)
        if (currSection.Name.Equals("Network") || currSection.Name.Equals("Ports") || currSection.Name.Equals("OverridePorts"))
        {
            //reload scene
            SceneManager.LoadScene(SceneManager.GetActiveScene().name);
        }
        else
        {
            SettingsManager.Instance.LoadConfigFromDisk(); // reload config
        }       
        
    }

    public void OnButton_CancelData()
    {
        //reload this section 
        SwitchToSection(currSection.Name);
    }

    public override void OnTabEnable()
    {
        OutputHelper.OutputLog("Settings Tab Enabled");
        //dont show message here
        MessageCanvas.SetActive(false);
    }

    public override void OnTabDisable()
    {
        OutputHelper.OutputLog("Settings Tab Disabled");
        //dont show message here
        MessageCanvas.SetActive(true);
    }

    public override RequiredSetting[] GetAllRequiredSettings()
    {
        //dont require anything for settings Tab
        return new RequiredSetting[0];
    }
}
