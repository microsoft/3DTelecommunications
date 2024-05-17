using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using UnityEngine;
using UnityEngine.UI;
using Image = UnityEngine.UI.Image;
using Color = UnityEngine.Color;

public abstract class TabController: MonoBehaviour
{
    public abstract void OnTabEnable();
    public abstract void OnTabDisable();
    
    public abstract RequiredSetting[] GetAllRequiredSettings();
    //returns if tab has all requirements met
    public virtual bool CheckTabPrerequisites(RequiredSetting[] requiredSettings, out List<string> errorMessages)
    {
        errorMessages = new List<string>();
        bool meetsAllPrerequisites = true;
        for (int i = 0; i < requiredSettings.Length; ++i)
        {
            if (!SettingsManager.Instance.config.Contains(requiredSettings[i].Section, requiredSettings[i].ParameterName))
            {
                string message = String.Format("Missing Config Value: [{0}][{1}]", requiredSettings[i].Section, requiredSettings[i].ParameterName);
                errorMessages.Add(message);
                OutputHelper.OutputLog(message);
                meetsAllPrerequisites = false;
            }
            else
            {
                if(requiredSettings[i].IsDirectory)
                {
                    string path = SettingsManager.Instance.GetValueWithDefault(requiredSettings[i].Section, requiredSettings[i].ParameterName, "", true);
                    if(path != "" && !System.IO.Directory.Exists(path))
                    {
                        if(requiredSettings[i].CreateIfNotExists)
                        {
                            System.IO.Directory.CreateDirectory(path);
                        }
                        else
                        {
                            string message = String.Format("Non-existent Folder [{0}][{1}] {2}", requiredSettings[i].Section, requiredSettings[i].ParameterName, path);
                            errorMessages.Add(message);
                            OutputHelper.OutputLog(message);
                            meetsAllPrerequisites = false;
                        }
                    }                    
                }
            }
        }
        return meetsAllPrerequisites;
    }

}

public class RequiredSetting
{
    public string Section { get; set; }
    public string ParameterName { get; set; }
    public string DefaultValue { get; set; }
    public bool IsDirectory { get; set; }
    public bool CreateIfNotExists { get; set; }
    public RequiredSetting(string section, string parameter, bool directory = false, bool create = false)
    {
        Section = section;
        ParameterName = parameter;
        IsDirectory = directory;
        CreateIfNotExists = create;
    }
}

[Serializable]
public class Tab
{
    //object to be activated by this tab
    public CanvasGroup RepresentedCanvasGroup;
    public Canvas RepresentedCanvas;
    public GameObject RepresentedCanvasGO;
    public TabController TabController;


    public Button TabTriggerButton;
    //Actual tab UI represenation
    public Image Icon;
    public Image UnderscoreImage;
    public Text Text;
}
public class ControlPanelTabManager : MonoBehaviour
{
    public Color TabSelectedColor = Color.blue;
    public Color TabIdleColor = Color.gray;
    public Color TabErrorColor = Color.red;

    [Header("UI Links")]
    public CanvasGroup TabOverlayCanvas;
    public GameObject overlayListContent;
    public GameObject listingPrefab;

    // drag and drop tabs in editor OR load from config
    public Tab[] allTabs;
    public int debugTabID;

    private int currentTabId = -1;

    private void Start()
    {
        if (allTabs != null)
        {
            for (int i = 0; i < allTabs.Length; ++i)
            {
                int currTabIndex = i;
                allTabs[i].TabTriggerButton.onClick.AddListener(() => { 
                    OnSelectTab(currTabIndex); 
                });

                if(allTabs[i].RepresentedCanvas == null)
                {
                    allTabs[i].RepresentedCanvas = allTabs[i].RepresentedCanvasGO.GetComponent<Canvas>();
                }
                if(allTabs[i].RepresentedCanvasGroup == null)
                {
                    allTabs[i].RepresentedCanvasGroup = allTabs[i].RepresentedCanvasGO.GetComponent<CanvasGroup>();
                }
            }
            // auto select a tab on startup
            resetUIStates(1);
        }        
        OnSelectTab(0); // select the 1st tab

        //disable debug tab if we need to
        if(!SettingsManager.Instance.GetValueWithDefault("UI","EnableDebugTab", false ))
        {
            allTabs[debugTabID].RepresentedCanvasGO.SetActive(false);
            allTabs[debugTabID].TabTriggerButton.gameObject.SetActive(false);
        }
    }
    
    void SetTabColor(int tabID, Color tabColor)
    {
        allTabs[tabID].UnderscoreImage.color = tabColor;
        allTabs[tabID].Icon.color = tabColor;
        allTabs[tabID].Text.color = tabColor;
    }

    void resetUIStates(int idToKeepActive = -1)
    {
        for(int i =0; i < allTabs.Length; ++i)
        {
            if (allTabs[i].TabController != null) {
                //keep this one active, so dont deactivate it
                if(i == idToKeepActive)
                {
                    // auto set visibility on startup
                    allTabs[i].RepresentedCanvas.enabled = true;
                    allTabs[i].RepresentedCanvasGroup.blocksRaycasts = true;
                    SetTabColor(i, TabSelectedColor);
                    //allTabs[i].TabController.OnTabEnable();
                }
                else
                {
                    allTabs[i].RepresentedCanvas.enabled= false;
                    allTabs[i].RepresentedCanvasGroup.blocksRaycasts = false;
                    SetTabColor(i, TabIdleColor);
                    //allTabs[i].TabController.OnTabDisable();
                }            
            }           
        }  
    }

    public void OnSelectTab(int currTabIndex)
    {
        if(currentTabId != -1)
        {
            allTabs[currentTabId].TabController.OnTabDisable();
        }
        currentTabId = currTabIndex;
        resetUIStates(currTabIndex);

        if(TabOverlayCanvas == null || overlayListContent== null || listingPrefab == null)
        {
            OutputHelper.OutputLog("Tab switch overlay not configured correctly!");
        }

        TabController currTabController = allTabs[currTabIndex].TabController;
        List<string> errorList;
        currTabController.OnTabEnable();
        if (!currTabController.CheckTabPrerequisites(currTabController.GetAllRequiredSettings(),out errorList))
        {
            SetTabColor(currTabIndex, TabErrorColor);
            //don't allow interaction with this tab's content with a barrier
            TabOverlayCanvas.alpha = 1;
            TabOverlayCanvas.blocksRaycasts = true;

            //remove previous errors
            foreach (Transform child in overlayListContent.transform)
            {
                GameObject.Destroy(child.gameObject);
            }
            //USE THE ERROR LIST TO POPULATE THE BOX
            for (int i =0; i < errorList.Count; ++i)
            {
                GameObject currObj = Instantiate(listingPrefab, overlayListContent.transform);
                Transform textTransform = currObj.transform.Find("textPanel/listingText");
                if(textTransform)
                {
                    Text currListingText = textTransform.GetComponent<Text>();
                    currListingText.text = errorList[i];
                }
            }
        }
        else
        {
            TabOverlayCanvas.alpha = 0;
            TabOverlayCanvas.blocksRaycasts = false;
        }        
    }

    public void OnButton_CloseTabOverlay()
    {
        TabOverlayCanvas.alpha = 0;
        TabOverlayCanvas.blocksRaycasts = false;
    }
}
