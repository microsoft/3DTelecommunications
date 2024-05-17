using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer.State.Actions;
using UnityEngine;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ViewerPanelBehavior: ViewerStateBehavior
    {
        [SerializeField]
        private ViewerPanel panel;

        protected override void OnDisable()
        {
            base.OnDisable();

            this.Dispatch(ViewerActionCreators.panelClosed(this.panel));
        }
    }
}
