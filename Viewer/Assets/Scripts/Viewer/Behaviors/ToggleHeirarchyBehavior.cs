using System.Collections.Generic;
using UnityEngine;

namespace Assets.Scripts.Viewer.Behaviors
{
    [ExecuteInEditMode]
    public class ToggleHeirarchyBehavior: MonoBehaviour
    {
        private List<ChildHideFlags> flags;

        [SerializeField]
        private bool hideHeirarchy = true;

        private void OnValidate()
        {
            ToggleHeirarchy(hideHeirarchy);
        }

        private void OnEnable()
        {
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
