using Assets.Scripts.Common.UI;
using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer.State.Actions;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class NavigationPanelBehavior: ViewerStateBehavior
    {
        [SerializeField]
        private CameraPreviewsBehavior previewController;

        [SerializeField]
        private Button front3DViewButton;

        [SerializeField]
        private BetterToggleGroup viewSelectionToggle;

        private int selectedCamera = 0;

        protected override void OnEnable()
        {
            RegisterStateChangeHandler(n => n.ViewActiveCamera, OnActiveCameraChanged);
            RegisterStateChangeHandler(n => n.ViewActiveDimension, OnActiveDimensionChanged);

            var activeCam = GetStateProperty(prop => prop.ViewActiveCamera).Value;
            OnActiveCameraChanged(activeCam, activeCam);

            var activeDimension = GetStateProperty(prop => prop.ViewActiveDimension).Value;
            OnActiveDimensionChanged(activeDimension, activeDimension);

            base.OnEnable();

            if (viewSelectionToggle != null)
            {
                viewSelectionToggle.OnChange += OnViewSelected;
            }

            previewController.cameraSelected += OnCameraSelected;

            if (previewController != null && previewController.isActiveAndEnabled)
            {
                previewController.SelectedCamera = selectedCamera;
            }

            if (front3DViewButton != null)
            {
                front3DViewButton.onClick.AddListener(OnFront3DViewClick);
            }
        }

        protected override void OnDisable()
        {
            base.OnDisable();

            if (viewSelectionToggle != null)
            {
                viewSelectionToggle.OnChange -= OnViewSelected;
            }

            previewController.cameraSelected -= OnCameraSelected;

            if (front3DViewButton != null)
            {
                front3DViewButton.onClick.RemoveListener(OnFront3DViewClick);
            }
        }

        private void OnFront3DViewClick()
        {
            int viewId = GetViewId();
            Dispatch(ViewerActionCreators.setViewDimension(viewId, Dimension.ThreeD));
        }

        private void OnActiveCameraChanged(ViewCamera[] newCameras, ViewCamera[] oldCameras)
        {
            int viewId = GetViewId();
            previewController.SelectedCamera = newCameras.First(n => n.ViewId == viewId).CameraNum;
        }

        private void OnActiveDimensionChanged(ViewDimension[] newCameras, ViewDimension[] oldCameras)
        {
        }

        private void OnCameraSelected(int cameraId)
        {
            int viewId = GetViewId();
            Dispatch(ViewerActionCreators.setViewCamera(viewId, cameraId));
        }

        private void OnViewSelected(Toggle newActive)
        {
            int viewId = GetViewId();
            int activeCamera = GetStateProperty(n => n.ViewActiveCamera).Value.First(n => n.ViewId == viewId).CameraNum;
            if (previewController != null && previewController.isActiveAndEnabled)
            {
                previewController.SelectedCamera = activeCamera;
            }
        }

        /// <summary>
        /// Finds the view we are currently in
        /// </summary>
        /// <returns></returns>
        private int GetViewId()
        {
            ViewBehavior v = gameObject.GetComponentInParent<ViewBehavior>();
            return v.viewId;
        }
    }
}
