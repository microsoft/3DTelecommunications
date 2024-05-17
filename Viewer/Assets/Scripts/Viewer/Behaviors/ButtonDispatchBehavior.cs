using Assets.Scripts.Viewer.State.Actions;
using UnityEngine.UI;
using UnityEngine;
using Assets.Scripts.Uniflux;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ButtonDispatchBehavior: ViewerStateBehavior
    {
        [SerializeField]
        public Button buttonComponent;

        [SerializeField]
        public ViewerActionType action;

        public ButtonDispatchBehavior()
        {
        }

        protected override void OnEnable()
        {
            base.OnEnable();

            this.buttonComponent = this.buttonComponent != null ? this.buttonComponent : this.gameObject.GetComponent<Button>();
            if (buttonComponent == null)
            {
                buttonComponent = gameObject.GetComponentInChildren<Button>() as Button;
            }
            if (this.buttonComponent != null)
            {
                this.buttonComponent.onClick.AddListener(this.OnButtonClick);
            }
        }

        protected override void OnDisable()
        {
            base.OnDisable();

            if (this.buttonComponent != null)
            {
                this.buttonComponent.onClick.RemoveListener(this.OnButtonClick);
            }
        }

        private void OnButtonClick()
        {
            this.Dispatch(new UnifluxAction<ViewerActionType, object>(this.action));
        }
    }
}
