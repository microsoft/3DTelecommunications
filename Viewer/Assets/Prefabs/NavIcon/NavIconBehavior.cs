using System.Collections.Generic;
using TMPro;
using UnityEngine;

namespace Assets.Prefabs
{
    [ExecuteInEditMode]
    public class NavIconBehavior: MonoBehaviour
    {
        private List<ChildHideFlags> flags;

        [SerializeField]
        private string iconName;

        [SerializeField]
        private string title;

        [SerializeField]
        private bool hideHeirarchy = true;

        [SerializeField]
        private float iconRotationDeg = 0f;

        [SerializeField]
        private Color iconColor = Color.white;

        [SerializeField]
        private Color textColor = Color.white;

        private TextMeshProUGUI text;

        public Color TextColor
        {
            get
            {
                return textColor;
            }
            set
            {
                textColor = value;

                OnValidate();
            }
        }

        public Color IconColor
        {
            get
            {
                return iconColor;
            }
            set
            {
                iconColor = value;

                OnValidate();
            }
        }

        public string IconName
        {
            get
            {
                return iconName;
            }
            set
            {
                iconName = value;

                OnValidate();
            }
        }

        public string Title
        {
            get
            {
                return title;
            }
            set
            {
                title = value;

                OnValidate();
            }
        }

        private void OnEnable()
        {
            text = GetComponentInChildren(typeof(TextMeshProUGUI)) as TextMeshProUGUI;

            flags = new List<ChildHideFlags>();
            for (int i = 0; i < transform.childCount; i++)
            {
                GameObject obj = transform.GetChild(i).gameObject;
                flags.Add(new ChildHideFlags()
                {
                    child = obj,
                    hideFlags = obj.hideFlags
                });
            }

            OnValidate();
        }

        private void OnDisable()
        {
            ToggleHeirarchy(false);
        }

        private void OnValidate()
        {
            if (text != null)
            {
                string finalTitle = string.IsNullOrEmpty(title) ? "Untitled" : title;
                string finalIcon = string.IsNullOrEmpty(iconName) ? "default" : iconName;
                int idx = text.spriteAsset.GetSpriteIndexFromName(finalIcon);
                if (idx < 0)
                {
                    finalIcon = text.spriteAsset.spriteCharacterTable[0].name;
                }
                string finalColor = ColorUtility.ToHtmlStringRGBA(iconColor);
                string finalTextColor = ColorUtility.ToHtmlStringRGBA(textColor);
                text.text = $"<rotate={iconRotationDeg}><sprite name=\"{finalIcon}\" color=#{finalColor}></rotate>\n<size=40%><color=#{finalTextColor}>{finalTitle}</color></size>";
            }

            ToggleHeirarchy(hideHeirarchy);
        }

        private void ToggleHeirarchy(bool hide)
        {
            if (flags != null)
            {
                foreach (ChildHideFlags chf in flags)
                {
                    chf.child.hideFlags = hide ? HideFlags.HideInHierarchy : chf.hideFlags;
                }
            }
        }

        private class ChildHideFlags
        {
            public GameObject child;
            public HideFlags hideFlags;
        }
    }
}
