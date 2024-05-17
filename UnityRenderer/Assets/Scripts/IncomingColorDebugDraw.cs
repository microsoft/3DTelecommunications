using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class IncomingColorDebugDraw : MonoBehaviour
{
    public ReadAndProcessRawImage TargetDebugProcessImageScript;
    public RawImage incImage;
    public RawImage processedImage;
    public RawImage sharpenedImage;
    public RawImage resultImage;


    public bool shouldDraw = false;
    void Awake()
    {
        if(SettingsManager.Instance.config.Contains("Renderer", "DrawColorTextures"))
        {
            shouldDraw = SettingsManager.Instance.config["Renderer"]["DrawColorTextures"].BoolValue;

            //disable images if we're not drawing
            incImage.gameObject.SetActive(shouldDraw);
            processedImage.gameObject.SetActive(shouldDraw);
        }
       
    }
    // Start is called before the first frame update
    void Start()
    {
        if(TargetDebugProcessImageScript != null && shouldDraw)
        {
            incImage.texture = TargetDebugProcessImageScript.inputTextureArray;
            processedImage.texture = TargetDebugProcessImageScript.processedColorAtlas;
            sharpenedImage.texture = TargetDebugProcessImageScript.sharpenedColorAtlas;
            resultImage.texture = TargetDebugProcessImageScript.renderTextureAtlas;
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
