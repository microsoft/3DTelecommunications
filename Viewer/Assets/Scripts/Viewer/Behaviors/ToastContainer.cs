using Assets.Scripts.Common.UI;
using Assets.Scripts.Viewer.Common;
using Assets.Scripts.Viewer.Components;
using System;
using UnityEngine;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ToastContainer: MonoBehaviour
    {
        [SerializeField]
        public GameObject ToastPrefab;

        [SerializeField]
        public int fadeOutDurationMS = 1000;

        [SerializeField]
        public int fadeOutDelayMS = 5000;

        public void AddToast(Toast toast, Action handleClick)
        {
            if (ToastPrefab != null)
            {
                ToastMessage message = Instantiate(this.ToastPrefab, this.gameObject.transform).GetComponent<ToastMessage>();
                message.Title = toast.Title;
                message.Content = toast.Contents;

                message.gameObject.AddComponent<CanvasGroup>();

                var fadeBeh = message.gameObject.AddComponent<FadeBehavior>();
                fadeBeh.delay = this.fadeOutDelayMS;
                fadeBeh.durationMS = this.fadeOutDurationMS;
                fadeBeh.startOpacity = 1.0f;
                fadeBeh.endOpacity = 0.0f;

                message.OnPointerEnter += () =>
                {
                    fadeBeh.StopFade();
                };

                message.OnPointerExit += () =>
                {
                    fadeBeh.RestartFade();
                };

                fadeBeh.OnFadeComplete += () =>
                {
                    this.RemoveToast(message);
                };

                message.OnClick += () =>
                {
                    this.RemoveToast(message);
                    handleClick?.Invoke();
                };
            }
        }

        private void OnEnable()
        {
            ViewerManager.Current().Toaster.ToastContainer = this;
        }

        private void OnDisable()
        {
            ViewerManager.Current().Toaster.ToastContainer = null;
        }

        private void RemoveToast(ToastMessage message)
        {
            Destroy(message.gameObject);
        }
    }
}
