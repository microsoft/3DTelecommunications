using Assets.Scripts.Viewer.Models;
using UnityEngine;
using UnityEngine.UI;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class PanelContainerBehavior : ViewerStateBehavior
    {
        [SerializeField]
        private float panelVisibleBackgroundAlpha = 0.5f;

        protected override void OnEnable()
        {
            this.RegisterStateChangeHandler(state => state.VisiblePanel, this.OnVisiblePanelChanged);

            base.OnEnable();

            var visiblePanel = this.GetStateProperty(state => state.VisiblePanel).Value;
            this.OnVisiblePanelChanged(visiblePanel, visiblePanel);
        }

        private void OnVisiblePanelChanged(ViewerPanel state, ViewerPanel oldState)
        {
            // TODO: Think about whether or not to have a "ViewerPanel" type on each child panel, and then
            // hiding/showing which one should be visible, rather than searching by name.
            this.FindAnTogglePanel("ConnectPanel", state == ViewerPanel.Connect);
            this.FindAnTogglePanel("HelpPanel", state == ViewerPanel.Help);
            this.FindAnTogglePanel("SettingsPanel", state == ViewerPanel.Settings);
            this.FindAnTogglePanel("ModeSelectorPanel", state == ViewerPanel.Mode);
            this.FindAnTogglePanel("CamerasPanel", state == ViewerPanel.Navigation);
            this.FindAnTogglePanel("ExportPanel", state == ViewerPanel.Export);
        }

        private void FindAnTogglePanel(string name, bool state)
        {
            Transform panel = this.GetPanel(name);
            if (panel != null)
            {
                panel.gameObject.SetActive(state);
            }

            var bgImage = this.GetComponent<Image>();
            if (bgImage != null)
            {
                var panels = this.GetComponentsInChildren<ViewerPanelBehavior>();
                var color = bgImage.color;

                // This returns 1, (it includes ourself because we use a Canvas
                if (panels.Length == 0)
                {
                    bgImage.enabled = false;
                }
                else
                {
                    bgImage.enabled = true;
                    color.a = this.panelVisibleBackgroundAlpha;
                }
                bgImage.color = color;
            }
        }

        private Transform GetPanel(string name)
        {
            return this.transform.Find(name);
        }
    }
}
