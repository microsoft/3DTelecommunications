using Assets.Scripts.Common.UI;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace Assets.Prefabs.Tooltip
{
    public class TooltipDisplay : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler, IPointerDownHandler, IPointerUpHandler
    {
        [SerializeField]
        private GameObject tooltipPrefab;

        [SerializeField]
        private TooltipData tooltipData;

        [SerializeField]
        public int fadeInDelayMS = 2000;

        [SerializeField]
        public int fadeInDurationMS = 200;

        [SerializeField]
        public bool followMouse = false;

        private float offset = 10;

        private GameObject tooltip;

        public TooltipData Tooltip
        {
            get
            {
                return tooltipData;
            }
            set
            {
                tooltipData = value;
                UpdateTooltipText();
            }
        }

        private void OnDisable()
        {
            HideTooltip();
        }

        private void Update()
        {
            if (followMouse)
            {
                // Keep positioning the cursor next to the mouse
                MoveTooltip(Input.mousePosition);
            }
        }

        void IPointerEnterHandler.OnPointerEnter(PointerEventData eventData)
        {
            ShowTooltip(eventData.position);
        }

        void IPointerDownHandler.OnPointerDown(PointerEventData eventData)
        {
            if (tooltip == null)
            {
                ShowTooltip(eventData.position);
            }
        }

        void IPointerUpHandler.OnPointerUp(PointerEventData eventData)
        {
            if (!followMouse)
            {
                HideTooltip();
            }
        }

        void IPointerExitHandler.OnPointerExit(PointerEventData eventData)
        {
            HideTooltip();
        }

        protected void ShowTooltip(Vector3 position)
        {
            // Hide any previous tooltip
            HideTooltip();

            tooltip = Instantiate(tooltipPrefab);
            if (tooltip != null)
            {
                Canvas c = tooltip.GetComponent<Canvas>();
                c.renderMode = RenderMode.ScreenSpaceOverlay;

                UpdateTooltipFade();
                UpdateTooltipText();
                MoveTooltip(position);
            }
        }

        protected void MoveTooltip(Vector3 position)
        {
            if (tooltip != null)
            {
                RectTransform positionTransform = (RectTransform)tooltip.gameObject.transform.Find("TooltipContainer");
                LayoutRebuilder.ForceRebuildLayoutImmediate(positionTransform);

                positionTransform.transform.position = position;
                positionTransform.transform.Translate(offset, -offset, 0);

                Vector3 oldPos = positionTransform.transform.position;
                positionTransform.transform.position = new Vector3(
                    Mathf.Clamp(oldPos.x, 0, Screen.width - positionTransform.rect.width - 10),
                    Mathf.Clamp(oldPos.y, positionTransform.rect.height - 10, Screen.height - 10),
                    oldPos.z
                );
            }
        }

        protected void UpdateTooltipText()
        {
            if (tooltip != null)
            {
                var controller = tooltip.GetComponent<TooltipController>();
                controller.Title = tooltipData.Title;
                controller.Contents = tooltipData.Contents;
            }
        }

        protected void UpdateTooltipFade()
        {
            if (tooltip != null)
            {
                var fade = tooltip.GetComponent<FadeBehavior>();
                fade.delay = fadeInDelayMS;
                fade.durationMS = fadeInDurationMS;
            }
        }

        protected void HideTooltip()
        {
            if (tooltip != null)
            {
                Destroy(tooltip);
                tooltip = null;
            }
        }
    }
}
