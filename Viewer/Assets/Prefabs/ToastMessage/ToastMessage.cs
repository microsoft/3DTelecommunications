using System;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

namespace Assets.Scripts.Viewer.Components
{
    public class ToastMessage : MonoBehaviour, IPointerClickHandler, IPointerEnterHandler, IPointerExitHandler
    {
        [SerializeField]
        public Action OnClick;

        public Action OnPointerEnter;

        public Action OnPointerExit;

        [SerializeField]
        private Text titleComponent;

        [SerializeField]
        private Text contentComponent;

        public string Title
        {
            set
            {
                if (this.titleComponent)
                {
                    this.titleComponent.text = value;
                }
            }
        }

        public string Content
        {
            set
            {
                if (this.contentComponent)
                {
                    this.contentComponent.text = value;
                }
            }
        }

        void IPointerClickHandler.OnPointerClick(PointerEventData eventData)
        {
            this.OnClick?.Invoke();
        }

        void IPointerEnterHandler.OnPointerEnter(PointerEventData eventData)
        {
            this.OnPointerEnter?.Invoke();
        }

        void IPointerExitHandler.OnPointerExit(PointerEventData eventData)
        {
            this.OnPointerExit?.Invoke();
        }
    }
}
