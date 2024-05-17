using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Assets.Prefabs.Tooltip
{
    [ExecuteInEditMode]
    public class TooltipController : MonoBehaviour
    {
        [SerializeField]
        private string tooltipTitle;

        [SerializeField]
        [TextArea]
        private string tooltipContents;

        [Header("Prefab UI Binding (No Touchie)")]
        [SerializeField]
        private GameObject TooltipContainer;

        [SerializeField]
        private Text TooltipTitleText;

        [SerializeField]
        private Text TooltipContentText;

        private HideFlags defaultContainerFlags;

        public string Title
        {
            get
            {
                return tooltipTitle;
            } 
            set
            {
                tooltipTitle = value;
                UpdateTitle();
            }
        }

        public string Contents
        {
            get
            {
                return tooltipContents;
            }
            set
            {
                tooltipContents = value;
                UpdateContents();
            }
        }

        private void OnEnable()
        {
            if (TooltipContainer != null) {
                defaultContainerFlags = TooltipContainer.hideFlags;
                TooltipContainer.hideFlags = HideFlags.HideInHierarchy;
            }

            UpdateTitle();
            UpdateContents();
        }

        private void OnDisable()
        {
            if (TooltipContainer != null)
            {
                TooltipContainer.hideFlags = defaultContainerFlags;
            }
        }

        private void OnValidate()
        {
            UpdateTitle();
            UpdateContents();
        }

        private void UpdateTitle()
        {
            if (TooltipTitleText != null)
            {
                TooltipTitleText.text = tooltipTitle;
                TooltipTitleText.gameObject.SetActive(!string.IsNullOrWhiteSpace(tooltipTitle));
                UpdateLayout();
            }
        }

        private void UpdateContents()
        {
            if (TooltipContentText != null)
            {
                TooltipContentText.text = tooltipContents;
                TooltipContentText.gameObject.SetActive(!string.IsNullOrWhiteSpace(tooltipContents));
                UpdateLayout();
            }
        }

        private void UpdateLayout()
        {
            if (TooltipContainer != null)
            {
                // No need to show the tooltip if there isn't anything to show
                if (string.IsNullOrWhiteSpace(tooltipTitle) &&
                    string.IsNullOrWhiteSpace(tooltipContents))
                {
                    TooltipContainer.SetActive(false);
                }
                else
                {
                    TooltipContainer.SetActive(true);
                    RectTransform transform = TooltipContainer.GetComponent<RectTransform>();
                    if (transform != null)
                    {
                        LayoutRebuilder.ForceRebuildLayoutImmediate(transform);
                    }
                }
            }
        }
    }
}
