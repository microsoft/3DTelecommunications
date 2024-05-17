using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class CalibrateButtonBehavior : MonoBehaviour
{
    // Start is called before the first frame update
    void Awake()
    {
        bool isDebugMode = SettingsManager.Instance.GetValueWithDefault("Calibration", "DebugMode", false);
        // 
        gameObject.SetActive(isDebugMode);
        //GetComponent<Button>().enabled = isDebugMode;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
