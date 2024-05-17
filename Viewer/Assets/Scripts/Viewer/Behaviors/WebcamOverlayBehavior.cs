using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;

namespace Assets.Scripts.Viewer.Behaviors {

    public class WebcamOverlayBehavior : MonoBehaviour
    {
        [SerializeField]
        private RawImage display;

        private WebCamTexture webcamTexture;

        private void Start()
        {
            // If display isn't set, try to grab off the GameObject
            if (display == null)
            {
                display = GetComponent<RawImage>();
            }

            if (display == null)
            {
                Debug.LogError("Webcam display not set!");
            }
        }

        private void OnEnable()
        {
            if (display != null)
            {
                display.enabled = true;

                WebCamDevice[] devices = WebCamTexture.devices;
                if (devices.Length > 0)
                {
                    // Try to find the device by the search string
                    WebCamDevice device = devices.FirstOrDefault(n => n.name.Contains(SettingsManager.Instance.PresenterCameraNameSearchString));
                    if (device.name == null) {

                        // Couldn't find it, search by the fallback
                        int fallbackIdx = SettingsManager.Instance.PresenterFallbackCameraIndex;

                        // Does the fallback index exist
                        if (fallbackIdx < devices.Length) 
                        {
                            device = devices[fallbackIdx];
                        } 
                        else 
                        {
                            device = devices[0];
                            Debug.LogError($"Could not find a camera with index { fallbackIdx }, using 0 instead");
                        }
                    }

                    // Did we actually find a device?
                    if (device.name != null) 
                    {
                        // For some reason, we need to specify a resolution here, otherwise we get crashes.
                        var resolution = SettingsManager.Instance.PresenterCameraResolution;
                        webcamTexture = new WebCamTexture(device.name, (int)resolution.x, (int)resolution.y);

                        display.texture = webcamTexture;

                        webcamTexture.Play();

                        StartCoroutine(WaitForWebCamAndInitialize(webcamTexture));
                    } 
                    else 
                    {
                        Debug.LogError($"Could not find a camera to use!");
                    }
                } 
                else 
                {
                    Debug.LogError($"No cameras found on the system.");
                }
            }
        }

        private void OnDisable()
        {
            if (display != null)
            {
                display.enabled = false;
            }
            if (webcamTexture != null)
            {
                webcamTexture.Stop();
                webcamTexture = null;
            }
        }

        private IEnumerator WaitForWebCamAndInitialize(WebCamTexture webcamTexture)
        {
            while (webcamTexture.width < 100)
                yield return null;

            display.texture = webcamTexture;
        }
    }
}
