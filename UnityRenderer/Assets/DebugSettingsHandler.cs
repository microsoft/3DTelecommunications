using UnityEngine;
using UnityEngine.UI;

public class DebugSettingsHandler : MonoBehaviour
{

    [Header("Holoport Script")]
    [SerializeField]
    private HoloPortScript holoportScript;

    [SerializeField]
    private Toggle streamToViewer;

    [SerializeField]
    private Slider processingPoolSize;

    [Header("Model To Texture")]
    [SerializeField]
    private HoloportModelToTexture modelToTexture;

    private void Start()
    {
        if (processingPoolSize != null)
        {
            processingPoolSize.value = SettingsManager.Instance.FusionNetworkProcessingPoolSize;
        }
    }

    // Update is called once per frame
    private void Update()
    {
        if (holoportScript != null)
        {
            if (streamToViewer != null)
            {
                holoportScript.renderUVSpace = streamToViewer.isOn;
            }

            if (processingPoolSize != null)
            {
                holoportScript.SetProcessingPoolSize((int)processingPoolSize.value);
            }
        }
    }
}
