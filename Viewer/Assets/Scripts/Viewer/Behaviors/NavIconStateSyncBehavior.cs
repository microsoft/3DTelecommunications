using Assets.Scripts.Common;
using Assets.Scripts.Viewer.State;
using Assets.Scripts.Viewer.State.Actions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.UIElements;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class NavIconStateSyncBehavior: ViewerStateBehavior
    {
        [SerializeField]
        public BoolViewerStateProperty stateProperty;

        [SerializeField]
        public Assets.Prefabs.NavIconBehavior navIcon;

        [Header("Settings if true")]
        [SerializeField]
        public Color trueColor;

        [SerializeField]
        public string trueText;

        [SerializeField]
        public string trueIconName;

        [Header("Settings if false")]
        [SerializeField]
        public Color falseColor;

        [SerializeField]
        public string falseText;

        [SerializeField]
        public string falseIconName;

        protected override void OnEnable()
        {
            RegisterStateChangeHandler(n => stateProperty.Property, OnPropertyChanged);

            base.OnEnable();

            bool value = stateProperty.Property.Value;
            OnPropertyChanged(value, value);
        }

        protected override void OnDisable()
        {
            base.OnDisable();
        }

        public void OnPropertyChanged(bool newValue, bool oldValue)
        {
            navIcon.TextColor = newValue ? trueColor : falseColor;
            navIcon.IconColor = newValue ? trueColor : falseColor;
            navIcon.Title = newValue ? trueText : falseText;

            string icon = newValue ? trueIconName : falseIconName;
            if (!string.IsNullOrWhiteSpace(icon)) {
                navIcon.IconName = icon;
            }
        }
    }
}
