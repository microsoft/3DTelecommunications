using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer.State.Actions;
using UnityEngine;
using UnityEngine.UI;
using Assets.Scripts.Common.Extensions;
using Ruffles.Connections;
using System.Data;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ConnectPanelBehavior: ViewerStateBehavior
    {
        [SerializeField]
        private Button connectButton;

        [SerializeField]
        private Button disconnectButton;

        [Header("3D Status")]
        [SerializeField]
        private Text connectionStatus3DText;

        [SerializeField]
        private GameObject connectionStatus3DConnectedIcon;

        [SerializeField]
        private GameObject connectionStatus3DConnectingIcon;

        [Header("2D Status")]
        [SerializeField]
        private Text connectionStatus2DText;

        [SerializeField]
        private GameObject connectionStatus2DConnectedIcon;

        [SerializeField]
        private GameObject connectionStatus2DConnectingIcon;


        private Dropdown connectionTargetDropDown;

        protected override void OnEnable()
        {
            RegisterStateChangeHandler(state => state.Holoport3DConnectionStatus, this.On3DStatusChanged);
            RegisterStateChangeHandler(state => state.Color2DConnectionStatus, this.On2DStatusChanged);
            RegisterStateChangeHandler(state => state.OverallConnectionStatus, this.OnOverallStatusChanged);

            base.OnEnable();

            connectButton.onClick.AddListener(this.OnConnectButtonClick);
            disconnectButton.onClick.AddListener(this.OnDisconnectButtonClick);

            ConnectionStatus status = this.GetStateProperty(state => state.Holoport3DConnectionStatus).Value;
            this.On3DStatusChanged(status, status);

            status = this.GetStateProperty(state => state.Color2DConnectionStatus).Value;
            this.On2DStatusChanged(status, status);

            status = GetStateProperty(n => n.OverallConnectionStatus).Value;
            OnOverallStatusChanged(status, status);
        }

        protected override void OnDisable()
        {
            base.OnDisable();

            connectButton.onClick.RemoveListener(this.OnConnectButtonClick);
            disconnectButton.onClick.RemoveListener(this.OnDisconnectButtonClick);
        }

        private void On3DStatusChanged(ConnectionStatus current, ConnectionStatus previous)
        {
            UpdateSubsystemStatus(current, connectionStatus3DText, connectionStatus3DConnectingIcon, connectionStatus3DConnectedIcon, true);
        }

        private void On2DStatusChanged(ConnectionStatus current, ConnectionStatus previous)
        {
            UpdateSubsystemStatus(current, connectionStatus2DText, connectionStatus2DConnectingIcon, connectionStatus2DConnectedIcon, false);
        }

        private void UpdateSubsystemStatus(ConnectionStatus subsystemStatus, Text statusText, GameObject connectingIcon, GameObject connectedIcon, bool is3D)
        {
            connectedIcon.SetActive(false);
            connectingIcon.SetActive(false);
            if (subsystemStatus == ConnectionStatus.Connecting)
            {
                this.SetStatusText("Connecting...", statusText);
                connectingIcon.SetActive(true);
            }
            else if (subsystemStatus == ConnectionStatus.Error)
            {
                this.SetStatusText($"Error: {this.GetStateProperty<string>(state => state.OverallConnectionError).Value}", statusText);
            }
            else if (subsystemStatus == ConnectionStatus.Connected)
            {
                connectedIcon.SetActive(true);
                this.SetStatusText($"Connected!", statusText);
            }
            else
            {
                this.SetStatusText("", statusText);
            }

            var mode = GetStateProperty(n => n.AppMode).Value;
            if (is3D)
            {
                // The status area is only available if we're in holoportation mode
                statusText.transform.parent.gameObject.SetActive(mode == ViewerMode.Holoportation);
            }
        }

        private void OnOverallStatusChanged(ConnectionStatus newStatus, ConnectionStatus status)
        {
            bool connectButtonEnabled = newStatus == ConnectionStatus.Disconnected || newStatus == ConnectionStatus.Error;
            disconnectButton.gameObject.SetActive(!connectButtonEnabled);
            connectButton.gameObject.SetActive(connectButtonEnabled);
        }


        private void OnConnectButtonClick()
        {
            Dispatch(ViewerActionCreators.startConnectionButtonClick(new InitConnectionData() {
                target = this.GetConnectionTarget()
            }));
        }

        private void OnDisconnectButtonClick()
        {
            Dispatch(ViewerActionCreators.disconnectButtonClick());
        }

        private string GetConnectionTarget()
        {
            if (this.connectionTargetDropDown != null)
            {
                var selectedOption = this.connectionTargetDropDown.value;
                return this.connectionTargetDropDown.options[selectedOption].text;
            }
            return null;
        }

        private void SetStatusText(string text, Text obj)
        {
            if (obj != null)
            {
                obj.text = text;
            }
        }
    }
}
