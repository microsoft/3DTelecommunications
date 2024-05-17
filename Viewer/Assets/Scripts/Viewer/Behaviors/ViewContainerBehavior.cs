using Assets.Scripts.Viewer.Models;
using System.Collections.Generic;
using System.Text.Encodings.Web;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ViewContainerBehavior : ViewerStateBehavior
    {
        protected override void OnEnable()
        {
            this.RegisterStateChangeHandler(state => state.DualView, this.OnDualViewChanged);
            this.RegisterStateChangeHandler(state => state.AppMode, this.OnModeChanged);
            this.RegisterStateChangeHandler(state => state.OverallConnectionStatus, this.OnConnectionStatusChanged);

            base.OnEnable();

            var dualView = this.GetStateProperty(state => state.DualView).Value;
            OnDualViewChanged(dualView, dualView);
            
            var status = GetStateProperty(state => state.OverallConnectionStatus).Value;
            OnConnectionStatusChanged(status, status);
        }

        private void OnModeChanged(ViewerMode mode, ViewerMode oldMode)
        {
            var dualView = this.GetStateProperty(state => state.DualView).Value;
            OnDualViewChanged(dualView, dualView);
        }

        private void OnConnectionStatusChanged(ConnectionStatus newStatus, ConnectionStatus oldStatus)
        {
            UpdateViews();
        }

        private void OnDualViewChanged(bool newDualView, bool oldDualView)
        {
            UpdateViews();
        }

        private void UpdateViews()
        {
            HashSet<int> viewsToEnable = new HashSet<int>();
            var status = GetStateProperty(state => state.OverallConnectionStatus).Value;
            if (status == ConnectionStatus.Connected)
            {
                viewsToEnable.Add(1);

                var dualView = this.GetStateProperty(state => state.DualView).Value;
                if (dualView)
                {
                    viewsToEnable.Add(2);
                }
            }

            var viewBehaviors = GetComponentsInChildren<ViewBehavior>(true);
            foreach (ViewBehavior beh in viewBehaviors)
            {
                beh.gameObject.SetActive(viewsToEnable.Contains(beh.viewId));
            }
        }
    }
}
