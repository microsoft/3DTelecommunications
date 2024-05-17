using Assets.Scripts.Events;
using Assets.Scripts.Viewer.State.Actions;
using Michsky.UI.ModernUIPack;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using System.Linq;
using Assets.Scripts.Viewer.Models;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ViewButtonManager : ViewerStateBehavior
    {
        [Header("Shared")]
        [SerializeField]
        private CameraControls cameraController;

        [SerializeField]
        private GameObject cameraSelectionPanel;

        [SerializeField]
        private ViewBehavior viewBehavior;

        [Header("Spotlight")]
        [SerializeField]
        private ViewButtonBehavior spotlightButton;

        [SerializeField]
        private SpotlightBehavior spotlightContainer;

        [Header("Misc")]
        [SerializeField]
        private ViewButtonBehavior resetViewButton;

        [SerializeField]
        private ViewButtonBehavior lookAtPatientButton;

        [SerializeField]
        private ViewButtonBehavior focusModeButton;

        [SerializeField]
        private ViewButtonBehavior drawToolButton;

        [SerializeField]
        private ViewButtonBehavior cameraSelectionButton;

        [SerializeField]
        private ViewButtonBehavior zoomToFitButton;

        [SerializeField]
        private ViewButtonBehavior zoomToLifesizeButton;

        [SerializeField]
        private SwitchManager dimensionToggleButton;

        /// <summary>
        /// List of eventDisconnects
        /// </summary>
        private List<Action> eventDisconnects = new List<Action>();

        /// <summary>
        /// Handler for when the behavior is enabled
        /// </summary>
        protected override void OnEnable()
        {
            RegisterStateChangeHandler(state => state.ViewActiveDimension, HandleViewActiveDimensionChanged);

            // Make sure the state of the dimension select button matches our application state
            HandleViewActiveDimensionChanged(GetStateProperty(state => state.ViewActiveDimension).Value, null);

            base.OnEnable();
            focusModeButton.Selected = false;
            spotlightButton.Selected = false;

            RegisterListener(cameraController.OnReticleModeChanged, (selected) => focusModeButton.Selected = selected);
            RegisterListener(spotlightButton.OnSelectedChanged, (selected) => spotlightContainer.enabled = selected);
            RegisterListener(focusModeButton.OnSelectedChanged, (selected) => cameraController.ReticleMode = selected);
            RegisterListener(resetViewButton.OnClick, () =>
            {
                cameraController.ResetCamera();
                spotlightButton.Selected = false;
                focusModeButton.Selected = cameraController.ReticleMode;
            });
            RegisterListener(lookAtPatientButton.OnClick, () => cameraController.LookAtFocusTarget());
            RegisterListener(drawToolButton.OnClick, () => cameraController.ToggleAnnotationMode());
            RegisterListener(cameraSelectionButton.OnSelectedChanged, selected => cameraSelectionPanel.SetActive(selected));
            RegisterListener(zoomToFitButton.OnClick, () => cameraController.ZoomToFit());
            RegisterListener(zoomToLifesizeButton.OnClick, () => cameraController.ZoomToLifeSize());
            RegisterListener(dimensionToggleButton.OnEvents, () => Dispatch(ViewerActionCreators.setViewDimension(viewBehavior.viewId, Dimension.ThreeD)));
            RegisterListener(dimensionToggleButton.OffEvents, () => Dispatch(ViewerActionCreators.setViewDimension(viewBehavior.viewId, Dimension.TwoD)));
        }

        /// <summary>
        /// Handler for when the behavior is disabled
        /// </summary>
        protected override void OnDisable()
        {
            base.OnDisable();

            foreach (var disconnect in eventDisconnects)
            {
                disconnect();
            }
            eventDisconnects.Clear();
        }

        /// <summary>
        /// <summary>
        /// Registers an event listener
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="evt"></param>
        /// <param name="listener"></param>
        private void RegisterListener<T>(UnityEvent<T> evt, UnityAction<T> listener)
        {
            evt.AddListener(listener);
            eventDisconnects.Add(() => evt.RemoveListener(listener));
        }

        /// <summary>
        /// Registers an event listener
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="evt"></param>
        /// <param name="listener"></param>
        private void RegisterListener(UnityEvent evt, UnityAction listener)
        {
            evt.AddListener(listener);
            eventDisconnects.Add(() => evt.RemoveListener(listener));
        }

        /// <summary>
        /// Handler for when the view active dimension is changed
        /// </summary>
        /// <param name="newVAD"></param>
        /// <param name="oldVAD"></param>
        private void HandleViewActiveDimensionChanged(ViewDimension[] newVAD, ViewDimension[] oldVAD) {
            Dimension newD = newVAD.First(n => n.ViewId == viewBehavior.viewId).Dimension;
            bool is3D = newD == Dimension.ThreeD;
            dimensionToggleButton.isOn = is3D;
            dimensionToggleButton.AnimateSwitch();
        }
    }
}