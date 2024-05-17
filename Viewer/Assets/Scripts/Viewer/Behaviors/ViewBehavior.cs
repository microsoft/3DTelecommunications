using UnityEngine;
using UnityEngine.UI;
using Assets.Scripts.Viewer.Models;
using System.Linq;
using Assets.Scripts.Common.Extensions;
using HoloportationDecoders.Color2D;
using HoloportationDecoders.Holoportation;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ViewBehavior : ViewerStateBehavior
    {
        [SerializeField]
        public GameObject[] threeDOnlyObjects;

        [SerializeField]
        public GameObject[] twoDOnlyObjects;

        [SerializeField]
        public Text debugTextTabel;

        [SerializeField]
        public int viewId;

        [Header("2D Configuration")]
        [SerializeField]
        public GameObject container2D;
        public RawImage output2D;

        [Header("3D Configuration")]
        [SerializeField]
        public GameObject container3D;
        public RawImage output3D;

        /// <summary>
        /// This is exposed in the inspector so it can be changed via the inspector
        /// </summary>
        [Header("Cameras")]
        [SerializeField]
        public int cameraNumOverride = -1;

        //[SerializeField]
        //public int frontCameraNum = 0;

        // ViewerState controlled values
        private int cameraNum = 0;
        public Dimension viewDimension = Dimension.TwoD;
        private ICameraFeed activeFeed;

        private Color2DDecoder cameraStreamProvider;
        private HoloportDecoder holoportDecoder;

        public int CameraNum
        {
            get
            {
                return cameraNum;
            }
            set
            {
                cameraNum = value;

                UpdateView();
            }
        }

        public Texture CurrentTexture
        {
            get
            {
                if (viewDimension == Dimension.TwoD)
                {
                    return output2D?.texture;
                } else
                {
                    return output3D?.texture;
                }
            }
        }

        protected override void OnEnable()
        {
            // Pull the correct camera number from the state
            RegisterStateChangeHandler(n => n.ViewActiveCamera, this.OnActiveCameraChanged);
            RegisterStateChangeHandler(n => n.ViewActiveDimension, this.OnActiveDimensionChanged);

            base.OnEnable();

            cameraStreamProvider = ViewerManager.Current().color2DDecoder;
            holoportDecoder = ViewerManager.Current().holoportDecoder;

            var activeCam = GetStateProperty(prop => prop.ViewActiveCamera).Value;
            OnActiveCameraChanged(activeCam, activeCam);

            var activeDimension = GetStateProperty(prop => prop.ViewActiveDimension).Value;
            OnActiveDimensionChanged(activeDimension, activeDimension);

            UpdateView();
        }


        void Update()
        {
            #if UNITY_EDITOR
            if (cameraNumOverride != cameraNum && cameraNumOverride >= 0)
            {
                cameraNum = cameraNumOverride;
                UpdateView();
            }
            #endif

            if (debugTextTabel != null && SettingsManager.Instance.ShowFPS)
            {
                // Enable the text label
                if (!debugTextTabel.isActiveAndEnabled)
                {
                    debugTextTabel.enabled = true;
                    debugTextTabel.transform.parent.gameObject.SetActive(true);
                }

                if (viewDimension == Dimension.ThreeD)
                {
                    debugTextTabel.text = $"~{holoportDecoder.FPS.ToString("0.00")} fps";
                }
                else if (activeFeed != null)
                {
                    debugTextTabel.text = $"~{activeFeed.ActualFPS.ToString("0.00")} fps";
                }
            }
        }

        /// <summary>
        /// Creates a PNG snapshot of the current view
        /// </summary>
        /// <returns>The png encoded byte[] arrray</returns>
        public byte[] Snapshot()
        {
            RectTransform output2DTransform = (RectTransform)this.output2D.transform;
            RectTransform scrollableTransform = (RectTransform)output2DTransform.parent;

            RawImage image = output2DTransform.GetComponent<RawImage>();
            if (image && image.texture)
            {
                Vector2 rawImageSize = new Vector2(image.texture.width, image.texture.height);

                Vector2 contentScreenSize = scrollableTransform.rect.size / scrollableTransform.localScale;
                Vector2 contentScreenTranslate = scrollableTransform.localPosition / scrollableTransform.localScale.x;
                Vector2 pivotOffset = scrollableTransform.pivot * scrollableTransform.rect.size;

                /* Imagine the outer box is the window of the scrollview
                 * The inner square is where the user has zoom and panned to
                 * TL is the top left point
                 * BR is the bottom right point
                 * both of these are the portion of the screen that the user was looking at if the content view wasn't scaled.

                 ----------------
                 |              |
                 |      TL      |
                 |       ----   |
                 |       |  |   |
                 |       ----   |
                 |            BR|
                 ----------------
                */
                Vector2 topLeft = new Vector2(
                    -contentScreenTranslate.x + pivotOffset.x,
                    (contentScreenTranslate.y + scrollableTransform.rect.height) - pivotOffset.y
                ) - output2DTransform.offsetMin;
                Vector2 bottomRight = topLeft + contentScreenSize;

                // Maps the TL point and the BR point onto the actual image
                var outputGameObjectToImageScale = (rawImageSize / output2DTransform.rect.size);
                Vector2 unclampedImageTopLeft = topLeft * outputGameObjectToImageScale;
                Vector2 unclampedImageBottomRight = bottomRight * outputGameObjectToImageScale;
                Vector2 outputImageTopLeft = new Vector2(
                    Mathf.Clamp(unclampedImageTopLeft.x, 0, rawImageSize.x),
                    Mathf.Clamp(unclampedImageTopLeft.y, 0, rawImageSize.y)
                );
                Vector2 outputBottomRight = new Vector2(
                    Mathf.Clamp(unclampedImageBottomRight.x, 0, rawImageSize.x),
                    Mathf.Clamp(unclampedImageBottomRight.y, 0, rawImageSize.y)
                );

                Texture2D cropped = ((Texture2D)image.texture).Crop(
                    (int)outputImageTopLeft.x,
                    (int)outputImageTopLeft.y,
                    (int)(outputBottomRight.x - outputImageTopLeft.x),
                    (int)(outputBottomRight.y - outputImageTopLeft.y),
                    output2DTransform.localScale.y < 0
                );

                return cropped.EncodeToPNG();
            }
            return null;
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            CleanView();
        }

        private void UpdateView()
        {
            if (isActiveAndEnabled)
            {
                // If the mode is TwoD, force the "front" camera.
                ViewerMode mode = GetStateProperty(prop => prop.AppMode).Value;
                int newCameraNum = GetStateProperty(prop => prop.ViewActiveCamera).Value.First(n => n.ViewId == viewId).CameraNum;
                int newCameraNumOverride = cameraNumOverride;
                Dimension newViewDimension = GetStateProperty(prop => prop.ViewActiveDimension).Value.First(n => n.ViewId == viewId).Dimension;

                if (mode == ViewerMode.Traditional)
                {
                    //cameraNum = frontCameraNum;
                    newCameraNum = SettingsManager.Instance.Default2DCamera;
                }
                else if (viewDimension == Dimension.ThreeD)
                {
                    newCameraNum = SettingsManager.Instance.Default2DCamera;
                    newCameraNumOverride = -1;
                }

                bool viewChanged =
                    newViewDimension != viewDimension ||
                    newCameraNum != cameraNum;

                viewDimension = newViewDimension;
                cameraNum = newCameraNum;
                cameraNumOverride = newCameraNumOverride;

                CleanView();

                // This component should *ALWAYS* be here
                var scrollable = container2D.GetComponent<PinchableScrollRect>();
                scrollable.zoomEnabled = SettingsManager.Instance.ZoomEnabled;
                scrollable.panEnabled = SettingsManager.Instance.PanEnabled;
                if (viewDimension == Dimension.TwoD)
                {
                    if (cameraStreamProvider != null && output2D != null)
                    {
                        var viewer = output2D.GetComponent<CameraFeedViewer>();
                        if (viewer != null)
                        {
                            activeFeed = cameraStreamProvider.CreateCameraFeed(cameraNum, SettingsManager.Instance.Default2DUpdateFPS);
                            viewer.Feed = activeFeed;

                            var crops = SettingsManager.Instance.CameraCrops;
                            Rect crop = Rect.zero;
                            if (crops == null || !crops.TryGetValue(cameraNum, out crop))
                            {
                                crop = Rect.zero;
                            }
                         
                            viewer.Crop = crop;
                        }
                        if (viewChanged)
                        {
                            scrollable.ResetState();
                        }
                    }
                }

                if (threeDOnlyObjects != null)
                {
                    foreach (GameObject obj in threeDOnlyObjects)
                    {
                        obj.SetActive(viewDimension == Dimension.ThreeD);
                    }
                }

                if (twoDOnlyObjects != null)
                {
                    foreach (GameObject obj in twoDOnlyObjects)
                    {
                        obj.SetActive(viewDimension == Dimension.TwoD);
                    }
                }
            }
        }

        private void OnActiveCameraChanged(ViewCamera[] newCameras, ViewCamera[] oldCameras)
        {
            UpdateView();
        }

        private void OnActiveDimensionChanged(ViewDimension[] newCameras, ViewDimension[] oldCameras)
        {
            UpdateView();
        }

        private void CleanView()
        {
            if (activeFeed != null)
            {
                activeFeed.Dispose();
                activeFeed = null;
            }

            if (output2D != null)
            {
                output2D.texture = null;

                var viewer = output2D.GetComponent<CameraFeedViewer>();
                if (viewer != null)
                {
                    viewer.Feed = null;
                }
            }

            bool isTwoD = viewDimension == Dimension.TwoD;
            container2D.SetActive(isTwoD);
            container3D.SetActive(!isTwoD);
        }
    }
}