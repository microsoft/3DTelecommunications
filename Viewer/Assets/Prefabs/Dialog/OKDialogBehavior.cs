using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

namespace Assets.Prefabs.Dialog
{
    public class OKDialogBehavior : MonoBehaviour
    {
        [SerializeField]
        private Button[] closeButtons;

        [SerializeField]
        private Button[] cancelButtons;

        protected void OnEnable()
        {
            this.AddListeners(cancelButtons, this.OnCancelButtonClick);
            this.AddListeners(closeButtons, this.OnCloseButtonClick);
        }
        protected void OnDisable()
        {
            this.RemoveListeners(cancelButtons, this.OnCancelButtonClick);
            this.RemoveListeners(closeButtons, this.OnCloseButtonClick);
        }
        protected void OnCancelButtonClick()
        {
            this.gameObject.SetActive(false);
        }

        protected void OnCloseButtonClick()
        {
            this.gameObject.SetActive(false);
        }

        private void AddListeners(Button[] buttonList, UnityAction listener)
        {
            if (buttonList != null)
            {
                foreach (Button b in buttonList)
                {
                    b.onClick.AddListener(listener);
                }
            }
        }

        private void RemoveListeners(Button[] buttonList, UnityAction listener)
        {
            if (buttonList != null)
            {
                foreach (Button b in buttonList)
                {
                    b.onClick.AddListener(listener);
                }
            }
        }
    }
}
