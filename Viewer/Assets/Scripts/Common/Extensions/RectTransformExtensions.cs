using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Assets.Scripts.Common.Extensions
{
    public static class RectTransformExtensions
    {
        // not threadsafe
        private static Vector3[] corners = new Vector3[4];
        private static Vector3[] corners2 = new Vector3[4];
        private const int TOP_LEFT_CORNER_IDX = 1;
        private const int BOTTOM_RIGHT_CORNER_IDX = 3;

        /// <summary>
        /// Clamps the __transform__ to the bounds defined by __clampTo__
        /// </summary>
        /// <param name="transform"></param>
        /// <param name="clampTo"></param>
        public static void ClampSize(this RectTransform transform, RectTransform clampTo)
        {
            if (transform != null && clampTo != null)
            {
                Vector2 sizeOffset = Vector2.zero;
                if (transform.rect.width > clampTo.rect.width)
                {
                    sizeOffset.x = clampTo.rect.width - transform.rect.width;
                }

                if (transform.rect.height > clampTo.rect.height)
                {
                    sizeOffset.y = clampTo.rect.height - transform.rect.height;
                }

                transform.sizeDelta += sizeOffset;
            }
        }

        /// <summary>
        /// Clamps the __transform__ to the bounds defined by __clampTo__
        /// </summary>
        /// <param name="transform"></param>
        /// <param name="clampTo"></param>
        public static void ClampPosition(this RectTransform transform, RectTransform clampTo)
        {
            if (transform != null && clampTo != null)
            {
                Vector3 offset = Vector3.zero;

                clampTo.GetWorldCorners(corners);
                transform.GetWorldCorners(corners2);

                // The top left corner when the the origin is the bottom left corner
                Vector3 clampTopLeft = corners[TOP_LEFT_CORNER_IDX];
                Vector3 clampBottomRight = corners[BOTTOM_RIGHT_CORNER_IDX];

                Vector2 transformTopLeft =
                    new Vector2(
                        Mathf.Min(corners2[0].x, corners2[1].x, corners2[2].x, corners2[3].x),
                        Mathf.Max(corners2[0].y, corners2[1].y, corners2[2].y, corners2[3].y)
                    );
                Vector2 transformBottomRight =
                    new Vector2(
                        Mathf.Max(corners2[0].x, corners2[1].x, corners2[2].x, corners2[3].x),
                        Mathf.Min(corners2[0].y, corners2[1].y, corners2[2].y, corners2[3].y)
                    );

                if (transformBottomRight.x > clampBottomRight.x)
                {
                    offset.x = clampBottomRight.x - transformBottomRight.x;
                }
                else if (transformTopLeft.x < clampTopLeft.x)
                {
                    offset.x = clampTopLeft.x - transformTopLeft.x;
                }

                if (transformBottomRight.y < clampBottomRight.y)
                {
                    offset.y = clampBottomRight.y - transformBottomRight.y;
                }
                else if (transformTopLeft.y > clampTopLeft.y)
                {
                    offset.y = clampTopLeft.y - transformTopLeft.y;
                }

                transform.position += offset;
            }
        }

        /// <summary>
        /// Clamps the __transform__ to the bounds defined by __clampTo__
        /// </summary>
        /// <param name="transform"></param>
        /// <param name="clampTo"></param>
        public static void Clamp(this RectTransform transform, RectTransform clampTo)
        {
            transform.ClampSize(clampTo);
            transform.ClampPosition(clampTo);
        }
    }
}
