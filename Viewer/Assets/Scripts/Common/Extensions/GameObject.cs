using UnityEngine;

namespace Assets.Scripts.Common.Extensions
{
    public static class GameObjectExtensions
    {
        public static T GetNestedComponentByName<T>(this Component obj, string name) where T : Component
        {
            return obj != null ? obj.gameObject.GetNestedComponentByName<T>(name) : null;
        }
        public static T GetNestedComponentByName<T>(this GameObject obj, string name) where T : Component
        {
            if (obj != null)
            {
                T component = obj.name == name ? obj.GetComponent<T>() : null;
                if (component != null)
                {
                    return component;
                }
                else
                {
                    Transform trans = obj.transform;
                    for (int i = 0; i < trans.childCount; i++)
                    {
                        Transform childTrans = trans.GetChild(i);
                        if (childTrans != null)
                        {
                            component = childTrans.gameObject.GetNestedComponentByName<T>(name);
                            if (component != null)
                            {
                                return component;
                            }
                        }
                    }
                }
            }
            return null;
        }

        public static void StretchToParent(this GameObject obj)
        {
            if (obj != null)
            {
                var rectTransform = obj.GetComponent<RectTransform>();
                if (rectTransform != null)
                {
                    rectTransform.offsetMax = new Vector2(0f, 0f);
                    rectTransform.offsetMin = new Vector2(0f, 0f);
                    rectTransform.anchorMin = new Vector2(0f, 0f);
                    rectTransform.anchorMax = new Vector2(1f, 1f);
                }
            }
        }

        /// <summary>
        /// Computes the center point of a game object
        /// If the game object has volume, it finds the center.
        /// </summary>
        /// <returns></returns>
        public static Vector3 ComputeCenter(this GameObject obj)
        {
            Bounds? b = obj.ComputeBounds();
            if (b != null)
            {
                return b.Value.center;
            }
            else
            {
                return obj.transform.position;
            }
        }

        public static Bounds? ComputeBounds(this GameObject obj)
        {
            Bounds? bounds = null;
            if (obj != null)
            {
                Renderer[] childRenderers = obj.GetComponentsInChildren<Renderer>(false);
                foreach (Renderer childRenderer in childRenderers)
                {
                    Bounds childBounds = childRenderer.bounds;
                    if (childBounds.size != Vector3.zero && childRenderer.enabled)
                    {
                        if (bounds == null)
                        {
                            bounds = childBounds;
                        }
                        else
                        {
                            Bounds b = bounds.Value;
                            b.Encapsulate(childBounds);
                            bounds = b;
                        }
                    }
                }
            }

            return bounds;
        }
    }
}
