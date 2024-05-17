using UnityEngine;
using UnityEngine.UI;

public class StatsDisplay : MonoBehaviour
{
    [Header("Holoport Script")]
    [SerializeField]
    private HoloPortScript holoportScript;

    [SerializeField]
    private Text holoportNetworkFPS;

    [SerializeField]
    private Text holoportNetworkSkippedFrames;

    [SerializeField]
    private Text holoportProcessingPoolCount;

    [SerializeField]
    private Text holoportComputeFPS;

    [Header("Model To Texture")]
    [SerializeField]
    private HoloportModelToTexture modelToTexture;

    [SerializeField]
    private Text modelToTextFPS;

    [SerializeField]
    private Text modelToTextMeshInfo;

    [Header("Viewer Handler")]
    [SerializeField]
    private ViewerHandler viewerHandler;

    [SerializeField]
    private Text numClients;

    [SerializeField]
    private Text network3DTransmitFPS;

    // Update is called once per frame
    void Update()
    {
        modelToTextFPS.text = modelToTexture.CurrentFPS.ToString();
        modelToTextMeshInfo.text = $"{modelToTexture.LastProcessedStruct.vertexCount} {modelToTexture.LastProcessedStruct.indexCount / 3}";

        holoportNetworkFPS.text = holoportScript.NetworkFPS.ToString();
        holoportNetworkSkippedFrames.text = holoportScript.NetworkSkippedFrames.ToString();
        holoportComputeFPS.text = holoportScript.ComputeFPS.ToString();
        holoportProcessingPoolCount.text = holoportScript.ProcessingPoolCount.ToString();

        numClients.text = viewerHandler.ClientCount.ToString();
        network3DTransmitFPS.text = viewerHandler.Network3DTransmitFPS.ToString();
    }
}
