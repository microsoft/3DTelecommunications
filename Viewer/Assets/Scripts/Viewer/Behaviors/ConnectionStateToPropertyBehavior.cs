using Assets.Scripts.Viewer.Models;
using System;
using UnityEngine;
using UnityEngine.Events;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ConnectionStateToPropertyBehavior : ViewerStateBehavior
    {
        [SerializeField]
        public ConnectionStatus ifModeEquals = ConnectionStatus.Disconnected;

        [SerializeField]
        public bool negate;

        [SerializeField]
        private BoolProperty PropertiesToUpdate;

        protected override void OnEnable()
        {
            RegisterStateChangeHandler(state => state.OverallConnectionStatus, OnConnectionStatusChanged);

            base.OnEnable();

            var status = this.GetStateProperty(state => state.OverallConnectionStatus).Value;
            this.OnConnectionStatusChanged(status, status);
        }

        private void OnConnectionStatusChanged(ConnectionStatus value, ConnectionStatus oldValue)
        {
            var areEqual = value == this.ifModeEquals;
            if (negate)
            {
                areEqual = !areEqual;
            }
            this.PropertiesToUpdate?.Invoke(areEqual);
        }

        [Serializable]
        private class BoolProperty : UnityEvent<bool> { }
    }
}
