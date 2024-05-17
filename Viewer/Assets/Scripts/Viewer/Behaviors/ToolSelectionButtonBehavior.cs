using Assets.Scripts.Common;
using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer.State.Actions;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ToolSelectionButtonBehavior: ViewerStateBehavior
    {
        [SerializeField]
        public ViewerTool tool;

        [SerializeField]
        public Button triggerButton;

        [SerializeField]
        public Color selectedColor;

        [SerializeField]
        public Behaviour toToggle;

        [SerializeField]
        public Assets.Prefabs.NavIconBehavior toolIcon;

        protected override void OnEnable()
        {
            RegisterStateChangeHandler(n => n.ActiveTool, OnActiveToolChanged);

            base.OnEnable();

            var activeTool = GetStateProperty(n => n.ActiveTool).Value;
            OnActiveToolChanged(activeTool, activeTool);

            if (triggerButton == null)
            {
                triggerButton = gameObject.GetComponentInChildren<Button>();
            }

            if (triggerButton != null) 
            {
                triggerButton.onClick.AddListener(OnButtonClick);
            }
        }

        protected override void OnDisable()
        {
            base.OnDisable();

            if (triggerButton != null)
            {
                triggerButton.onClick.RemoveListener(this.OnButtonClick);
            }
        }

        private void OnActiveToolChanged(ViewerTool newTool, ViewerTool oldTool)
        {
            if (toolIcon != null)
            {
                Color color = newTool == tool ? selectedColor : Color.white;
                toolIcon.TextColor = color;
                toolIcon.IconColor = color;
            }

            if (toToggle != null)
            {
                toToggle.enabled = newTool == tool;
            }
        }

        private void OnButtonClick()
        {
            Dispatch(ViewerActionCreators.activeToolSelected(tool));
        }
    }
}
