using Assets.Scripts.Common.Extensions;
using Assets.Scripts.Viewer.Models;
using UnityEngine;

namespace Assets.Scripts.Viewer {
    public class MainNavigationBehavior : ViewerStateBehavior
    {
        public GameObject zoomButton;
        public GameObject panButton;
        public GameObject orbitButton;

        /// <summary>
        /// The separators used for the zoom/pan mode buttons
        /// </summary>
        public GameObject[] toolSeparators;

        protected override void OnEnable()
        {
            // The tool states are dependent on both the app mode and connection status
            RegisterStateChangeHandler(n => n.OverallConnectionStatus, OnConnectionStatusChange);
            RegisterStateChangeHandler(n => n.AppMode, OnAppModeChanged);

            base.OnEnable();
        }

        private void OnConnectionStatusChange(ConnectionStatus cs, ConnectionStatus oldCs)
        {
            UpdateButtonStates();
        }

        private void OnAppModeChanged(ViewerMode newMode, ViewerMode oldMode)
        {
            UpdateButtonStates();
        }

        private void UpdateButtonStates()
        {
            bool isConnected = GetStateProperty(n => n.OverallConnectionStatus).Value == ConnectionStatus.Connected;
            bool anyToolEnabled = false;
            if (zoomButton != null)
            {
                bool toolEnabled = SettingsManager.Instance.ZoomEnabled && isConnected;
                zoomButton.SetActive(toolEnabled);

                anyToolEnabled = anyToolEnabled || toolEnabled;
            }
            if (panButton != null)
            {
                bool toolEnabled = SettingsManager.Instance.PanEnabled && isConnected;
                panButton.SetActive(toolEnabled);

                anyToolEnabled = anyToolEnabled || toolEnabled;
            }
            if (orbitButton != null)
            {
                bool toolEnabled = SettingsManager.Instance.PanEnabled && isConnected;
                orbitButton.SetActive(toolEnabled);

                anyToolEnabled = anyToolEnabled || toolEnabled;
            }

            if (toolSeparators != null)
            {
                foreach (GameObject separator in toolSeparators)
                {
                    separator.SetActive(anyToolEnabled);
                }
            }
        }
    }
}