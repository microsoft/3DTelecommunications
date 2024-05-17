using Assets.Scripts.Viewer.State;
using System;
using UnityEngine;
using UnityEngine.Events;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class SettingManagerBoolSetting : ViewerStateBehavior
    {
        [SerializeField]
        public string section;

        [SerializeField]
        public string setting;

        [SerializeField]
        public bool defaultValue;

        [SerializeField]
        [System.ComponentModel.Description("Syncs every time the Viewer State changes")]
        public bool highPriority;

        [SerializeField]
        public BoolProperty syncedProperties;
        
        protected override void OnEnable()
        {
            ViewerManager.Current().Store.LateStateChangeMiddleware += OnStateChanged;

            base.OnEnable();

            SyncProperties();
        }

        protected override void OnDisable()
        {
            base.OnDisable();

            ViewerManager.Current().Store.LateStateChangeMiddleware -= OnStateChanged;
        }

        private void OnStateChanged(ViewerState newState, ViewerState oldState, object changes)
        {
            if (highPriority)
            {
                SyncProperties();
            }
        }

        private void SyncProperties()
        {
         
            bool value = SettingsManager.Instance.GetOrDefault(section, setting, defaultValue);            
            syncedProperties?.Invoke(value);
        }

        [Serializable]
        public class BoolProperty : UnityEvent<bool> { }
    }
}
