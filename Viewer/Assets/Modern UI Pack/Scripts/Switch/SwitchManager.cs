using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;

namespace Michsky.UI.ModernUIPack
{
    public class SwitchManager : MonoBehaviour
    {
        // Events
        public UnityEvent OnEvents;
        public UnityEvent OffEvents;

        // Saving
        public bool saveValue = true;
        public string switchTag = "Switch";

        // Settings
        public bool isOn = true;
        public bool invokeAtStart = false;

        [HideInInspector] public Animator switchAnimator;
        [HideInInspector] public Button switchButton;

        void Start()
        {
            try
            {
                switchAnimator = gameObject.GetComponent<Animator>();
                switchButton = gameObject.GetComponent<Button>();
                switchButton.onClick.AddListener(() => {
                    isOn = !isOn;
                    AnimateSwitch();
                    TriggerEvents();
                });
            }

            catch
            {
                Debug.LogError("Switch - Cannot initalize the switch due to missing variables.", this);
            }

            //isOn = GetSwitchStateFromPrefs();

            AnimateSwitch();
            if (invokeAtStart)
            {
                TriggerEvents();
            }
        }

        void OnEnable()
        {
            if (switchAnimator == null)
                switchAnimator = gameObject.GetComponent<Animator>();

            AnimateSwitch();
        }

        public void AnimateSwitch()
        {
            string onOff = isOn ? "On" : "Off";
            string trueFalse = isOn ? "true" : "false";
            switchAnimator.Play($"Switch {onOff}");
            if (saveValue == true) {
                PlayerPrefs.SetString(switchTag + "Switch", trueFalse);
            }
        }

        private bool GetSwitchStateFromPrefs()
        {
            return PlayerPrefs.GetString(switchTag + "Switch") == "" || PlayerPrefs.GetString(switchTag + "Switch") == "true";
        }

        private void TriggerEvents()
        {
            if (isOn)
            {
                OnEvents.Invoke();
            } 
            else
            {
                OffEvents.Invoke();
            }
        }
    }
}