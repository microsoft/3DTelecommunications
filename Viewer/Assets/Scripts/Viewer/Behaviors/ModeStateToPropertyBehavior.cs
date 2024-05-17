using Assets.Scripts.Viewer.Models;
using System;
using UnityEngine;
using UnityEngine.Events;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ModeStateToPropertyBehavior : ViewerStateBehavior
    {
        [SerializeField]
        public ViewerMode ifModeEquals = ViewerMode.Traditional;

        [SerializeField]
        public bool negate;

        [SerializeField]
        public bool checkConnection = true;

        [SerializeField]
        private BoolProperty PropertiesToUpdate;

        protected override void OnEnable()
        {
            this.RegisterStateChangeHandler(state => state.AppMode, this.OnModeChanged);

            if (checkConnection)
            {
                // TODO: This is a workaround, because the navigation button needs to be disabled if
                // the viewer isn't connected AND the mode is not 2D, but there is no way to specify that
                // in the inspector
                this.RegisterStateChangeHandler(state => state.OverallConnectionStatus, this.OnConnectionStatusChanged);
            }

            base.OnEnable();

            var mode = this.GetStateProperty(state => state.AppMode).Value;
            this.OnModeChanged(mode, mode);
        }

        private void OnModeChanged(ViewerMode value, ViewerMode oldValue)
        {
            var areEqual = value == this.ifModeEquals;
            if (negate)
            {
                areEqual = !areEqual;
            }
            this.PropertiesToUpdate?.Invoke(areEqual && (!checkConnection || (GetStateProperty(n => n.OverallConnectionStatus).Value == ConnectionStatus.Connected)));
        }
        private void OnConnectionStatusChanged(ConnectionStatus value, ConnectionStatus oldValue)
        {
            var mode = this.GetStateProperty(state => state.AppMode).Value;
            this.OnModeChanged(mode, mode);
        }

        [Serializable]
        private class BoolProperty : UnityEvent<bool> { }
    }
}
