using Assets.Scripts.Viewer.State;
using Assets.Scripts.Viewer.Common;
using Assets.Scripts.Viewer.State.Actions;
using Assets.Scripts.Uniflux;
using IViewerStore = Assets.Scripts.Uniflux.IUnifluxStore<Assets.Scripts.Viewer.State.Actions.ViewerActionType, Assets.Scripts.Viewer.State.ViewerState>;
using UnityEngine;
using NetMQ;
using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer.State.Middleware;
using HoloportationDecoders.Holoportation;
using HoloportationDecoders.Color2D;

namespace Assets.Scripts.Viewer {

    public class ViewerManager: MonoBehaviour
    {
        [SerializeField]
        public HoloportDecoder holoportDecoder;

        [SerializeField]
        public HoloportRawFrameViewer hqHoloportViewer;

        [SerializeField]
        public Color2DDecoder color2DDecoder;

        [SerializeField]
        public Texture2D zoomCursor;

        [SerializeField]
        public Texture2D panCursor;

        [SerializeField]
        public Texture2D orbitCursor;

        [SerializeField]
        private Camera front3DCamera;

        [SerializeField]
        private Camera back3DCamera;

        [SerializeField]
        private Camera right3DCamera;

        [SerializeField]
        private Camera left3DCamera;

        [SerializeField]
        private Camera top3DCamera;

        [SerializeField]
        private MeshRenderer patientRoom;

        private static ViewerManager current;

        private bool IsControlDown = false;

        private void OnEnable()
        {
            if (current != null && current != this)
            {
                Debug.LogError("ViewerManager should only be attached once!");
                return;
            }

            if (color2DDecoder == null)
            {
                Debug.LogError($"Color2DDecoder should be attached to the viewer manager");
                return;
            }

            if (holoportDecoder == null)
            {
                Debug.LogError($"HoloportDecoder should be attached to the viewer manager");
                return;
            }

            if (hqHoloportViewer == null)
            {
                Debug.LogError($"HQHoloportDecoder should be attached to the viewer manager");
                return;
            }

            PositionCamera(front3DCamera, SettingsManager.Instance.Front3DCameraOffset);
            PositionCamera(back3DCamera, SettingsManager.Instance.Back3DCameraOffset);
            PositionCamera(left3DCamera, SettingsManager.Instance.Left3DCameraOffset);
            PositionCamera(right3DCamera, SettingsManager.Instance.Right3DCameraOffset);
            PositionCamera(top3DCamera, SettingsManager.Instance.Top3DCameraOffset);

            holoportDecoder.Settings = SettingsManager.Instance.ToDecoderSettings();
            color2DDecoder.Settings = SettingsManager.Instance.ToDecoderSettings();

            current = this;

            // This is for NetMQ, otherwise can cause unity to hang
            // https://github.com/zeromq/netmq/issues/324
            AsyncIO.ForceDotNet.Force();

            var state = new ViewerState(
                SettingsManager.Instance.DefaultViewerMode,
                SettingsManager.Instance.Default2DCamera
            );
            Store = new UnifluxStoreImpl<ViewerActionType, ViewerState>("Viewer", state);

            new ViewerReducers(Store);
            new ViewerEpics(
                Store,
                color2DDecoder,
                holoportDecoder,
                hqHoloportViewer,
                Toaster,
                zoomCursor,
                panCursor,
                orbitCursor,
                SettingsManager.Instance
            );
            new ActionLogger(Store);

            // TODO: This should happen in the store
            Store.Dispatch(new UnifluxAction<ViewerActionType, object>(ViewerActionType.Initialize));

            UpdatePatientRoomFromSettings();
        }

        private void OnDisable()
        {
            bool blocking = false;

            // Do not block because this gets called before other things can be destroyed
            NetMQConfig.Cleanup(blocking);
        }

        public IViewerStore Store
        {
            get;
            private set;
        }

        public virtual Toaster Toaster { 
            get;
            private set;
        } = new Toaster();

        public static ViewerManager Current()
        {
            return current;
        }

        private void Update()
        {
            var state = Store.GetState();
            if (Input.GetKey(KeyCode.Escape))
            {
                if (state.ActiveTool.Value != ViewerTool.None)
                {
                    Store.Dispatch(ViewerActionCreators.activeToolSelected(ViewerTool.None));
                }


                // TODO: This might should be controlled at the panel level
                // Close the current panel
                var currentPanel = state.VisiblePanel.Value;

                // Can't close the mode panel
                if (currentPanel != ViewerPanel.Mode && currentPanel != ViewerPanel.None)
                {
                    Store.Dispatch(ViewerActionCreators.panelClosed(currentPanel));
                }
            }

            else if (Utils.IsDeviceIndependentControlDown())
            {
                IsControlDown = true;
            }
            else if (Utils.IsDeviceIndependentControlUp())
            {
                IsControlDown = false;
            }

            if (state.OverallConnectionStatus.Value == ConnectionStatus.Connected)
            {
                // CTRL + S - Save view
                if (IsControlDown && Input.GetKeyDown(KeyCode.S))
                {
                    Store.Dispatch(ViewerActionCreators.snapshotButtonClick());
                }

                // CTRL + O - Open View
                else if (IsControlDown && Input.GetKeyDown(KeyCode.O))
                {
                    Store.Dispatch(ViewerActionCreators.navigationButtonClicked());
                }
            } 
            else if (state.OverallConnectionStatus.Value == ConnectionStatus.Disconnected)
            {
                // CTRL + N
                if (IsControlDown && Input.GetKeyDown(KeyCode.N))
                {
                    Store.Dispatch(ViewerActionCreators.connectButtonClick());
                }
            }
        }

        private void PositionCamera(Camera camera, Vector3 position)
        {
            if (camera != null)
            {
                camera.transform.localPosition = position;
            }
        }

        /// <summary>
        /// Updates the patient room to match the user settings
        /// </summary>
        private void UpdatePatientRoomFromSettings()
        {
            if (patientRoom)
            {
                if (!SettingsManager.Instance.RoomGrid)
                {
                    // This doesn't handle dynamic changes to the setting
                    // If the user doesn't want a grid, then use a basic white texture
                    patientRoom.material.mainTexture = Texture2D.whiteTexture;
                }

                if (SettingsManager.Instance.RoomShadows)
                {
                    // This doesn't handle dynamic changes to the setting
                    // If the user wants shadows, then we need to use a standard shader
                    patientRoom.material.shader = Shader.Find("Standard");
                }
            }

            // Update the "tint" of the material to match the setting
            patientRoom.material.SetColor("_Color", SettingsManager.Instance.RoomBackgroundColor);
        }
    }
}