using Assets.Scripts.Events;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ViewButtonBehavior : MonoBehaviour
    {
        private bool selected;

        [SerializeField]
        private bool isToggle;

        [SerializeField]
        private Button button;

        /// <summary>
        /// Event for when click events occur
        /// </summary>
        public UnityEvent OnClick;

        /// <summary>
        /// Event for when the selected state changes
        /// </summary>
        public BoolEvent OnSelectedChanged;

        public bool Selected
        {
            get { return selected; }
            set
            {
                if (value != selected) {
                    selected = value;
                    UpdateButtonState();
                    OnSelectedChanged?.Invoke(value);
                }
            }
        }

        /// <summary>
        /// Listeners for when the behavior is enabled
        /// </summary>
        private void OnEnable()
        {
            if (button == null)
            {
                button = GetComponent<Button>();
            }
            UpdateButtonState();

            OnSelectedChanged?.Invoke(Selected);

            button.onClick.AddListener(HandleButtonClick);
        }

        /// <summary>
        /// Listeners for when the behavior is disabled
        /// </summary>
        private void OnDisable()
        {
            Selected = false;
            button.onClick.RemoveListener(HandleButtonClick);
        }

        /// <summary>
        /// Syncs the button state with the behaviors state
        /// </summary>
        private void UpdateButtonState()
        {
            if (isToggle)
            {
                if (Selected)
                {
                    button.OnSelect(null);
                }
                else
                {
                    button.OnDeselect(null);
                }
            }
        }

        /// <summary>
        /// Event handler when the button is clicked on
        /// </summary>
        private void HandleButtonClick()
        {
            if (isToggle)
            {
                Selected = !Selected;
            }

            OnClick?.Invoke();
        }
    }
}


