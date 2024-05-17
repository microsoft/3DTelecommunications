using System;
using UnityEngine;

namespace Assets.Scripts.Common.UI
{
    public delegate void FadeHandler();

    public class FadeBehavior: MonoBehaviour
    {
        [SerializeField]
        public EaseType ease = EaseType.Linear;

        [SerializeField]
        public float startOpacity = 0;

        [SerializeField]
        public float endOpacity = 1;

        [SerializeField]
        public int durationMS = 1000;

        [SerializeField]
        public int delay = 0;

        [SerializeField]
        public FadeHandler OnFadeBegin;

        [SerializeField]
        public FadeHandler OnFadeComplete;

        private CanvasGroup group;

        private long animStart = -1;
        private bool notifyComplete = false;
        private bool paused = false;

        void Start() {
            this.group = this.gameObject.GetComponent<CanvasGroup>();
            this.animStart = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            this.notifyComplete = false;
            if (this.OnFadeBegin != null)
            {
                this.OnFadeBegin.Invoke();
            }
            if (this.delay == 0)
            {
                this.SetOpacity(this.startOpacity);
            }
        }

        void Update()
        {
            if (this.group != null && !this.paused)
            {
                long currentTime = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                if ((currentTime - this.animStart) > this.delay)
                {
                    float percent = Math.Max(0, Math.Min((float)((currentTime - this.animStart - this.delay) / (float)this.durationMS), 1));
                    if (percent < 1.0)
                    {
                        float eased = EaseFunction.Ease(percent, this.ease);
                        this.SetOpacity(this.startOpacity + ((this.endOpacity - this.startOpacity) * eased));
                    }
                    else if (!this.notifyComplete && this.OnFadeComplete != null)
                    {
                        this.notifyComplete = true;
                        this.OnFadeComplete.Invoke();
                    }
                }
            }
        }

        public void StopFade()
        {
            this.paused = true;
            this.SetOpacity(this.startOpacity);
        }

        public void RestartFade()
        {
            this.paused = false;
            this.animStart = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        }

        private void SetOpacity(float value)
        {
            if (this.group)
            {
                this.group.alpha = value;
            }
        }
    }
}
