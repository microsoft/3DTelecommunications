using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Events;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ViewerStatePropertySyncBehavior: ViewerStateBehavior
    {
        [SerializeField]
        public BoolViewerStateProperty stateProperty;

        [SerializeField]
        private BoolProperty propertiesToSync;

        protected override void OnEnable()
        {
            RegisterStateChangeHandler(n => stateProperty.Property, OnPropertyChanged);
            base.OnEnable();

            OnPropertyChanged(stateProperty.Property.Value, stateProperty.Property.Value);
        }

        private void OnPropertyChanged(bool newVal, bool oldVal)
        {
            propertiesToSync?.Invoke(newVal);
        }


        [Serializable]
        private class BoolProperty : UnityEvent<bool> { }
    }
}
