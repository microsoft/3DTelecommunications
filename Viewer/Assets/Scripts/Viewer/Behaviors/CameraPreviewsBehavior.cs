using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;
using HoloportationDecoders.Color2D;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class CameraPreviewsBehavior: MonoBehaviour
    {

        [SerializeField]
        private GameObject cameraPreviewContainer;

        [SerializeField]
        private GameObject cameraPreviewPrefab;

        [SerializeField]
        private Font previewTextFont;

        [SerializeField]
        private int previewTextFontSize = 12;

        [SerializeField]
        private bool showPreviewText = false;

        /// <summary>
        /// The update frequency of the previews
        /// </summary>
        [SerializeField]
        private float previewUpdateFPS = 0.5f; // every 2 seconds

        private int selectedCamera = 0;

        public event Action<int> cameraSelected;
        private List<CameraPreviewController> cameraPreviews;

        /// <summary>
        /// Reference to the color2d decoder instance
        /// </summary>
        private Color2DDecoder color2DDecoder;

        /// <summary>
        /// Gets/sets the selected camera
        /// </summary>

        public int SelectedCamera
        {
            get 
            { 
                return selectedCamera;
            }
            set
            {
                selectedCamera = value;
                UpdatePreviewSelectionState();
            }
        }

        /// <summary>
        /// Handler for when the behavior is enabled
        /// </summary>
        private void OnEnable()
        {
            color2DDecoder = ViewerManager.Current().color2DDecoder;
            if (color2DDecoder == null)
            {
                Debug.LogError("Color2DEncoder needs to be defined");
                return;
            }

            cameraPreviews = new List<CameraPreviewController>();
            int numCameras = SettingsManager.Instance.NumCameras;

            // Create previews for all the cameras we have available
            for (int i = 0; i < numCameras; i++)
            {
                CameraPreviewController preview = Instantiate(this.cameraPreviewPrefab, cameraPreviewContainer.transform)
                    .GetComponent<CameraPreviewController>();
                preview.transform.localScale = Vector3.one;
                preview.Feed = color2DDecoder.CreateCameraFeed(i, previewUpdateFPS);
                preview.OnClick.AddListener(() =>
                {
                    cameraSelected?.Invoke(preview.Feed.CameraIdx);
                });
                cameraPreviews.Add(preview);
            }

            SelectedCamera = selectedCamera;
        }
        
        /// <summary>
        /// Handler for when the behavior is disabled
        /// </summary>
        private void OnDisable()
        {
            if (cameraPreviews != null)
            {
                foreach (CameraPreviewController preview in cameraPreviews)
                {
                    preview.OnClick.RemoveAllListeners();

                    // Cleanup the feed
                    var feed = preview.Feed;
                    preview.Feed = null;
                    feed.Dispose();
                }
            }

            foreach (Transform child in transform)
            {
                Destroy(child.gameObject);
            }
            cameraPreviews = null;
        }

        /// <summary>
        /// Updates the preview's selection state to match the selected index
        /// </summary>
        private void UpdatePreviewSelectionState()
        {
            if (cameraPreviews != null)
            {
                foreach (CameraPreviewController preview in cameraPreviews)
                {
                    bool isSelected = preview.Feed.CameraIdx == selectedCamera;
                    preview.Selected = isSelected;
                }
            }
        }
    }
}
