using UnityEngine;
using UnityEngine.UI;

namespace Assets.Scripts.Common
{
    public class CameraImageFeedBehavior: MonoBehaviour
    {
        [SerializeField]
        private Camera imageSourceCamera;

        [SerializeField]
        private RawImage outputImage;

        [SerializeField]
        private float outputImageScale = 1.0f;

        [SerializeField]
        private FilterMode filterMode = FilterMode.Point;

        private float currentWidth = -1f;
        private float currentHeight = -1f;

        private RenderTexture outputTexture;
        private void Update()
        {
            if (HasSizeChanged())
            {
                if (outputTexture != null)
                {
                    outputTexture.Release();
                }

                var rectTransform = (RectTransform)transform;
                currentWidth = rectTransform.rect.width;
                currentHeight = rectTransform.rect.height;
                //cameraCopy.pixelRect = new Rect(0, 0, (int)rectTransform.rect.width, (int)rectTransform.rect.height);
                imageSourceCamera.aspect = rectTransform.rect.width / rectTransform.rect.height;
                outputTexture = RenderTexture.GetTemporary((int)(rectTransform.rect.width * outputImageScale), (int)(rectTransform.rect.height * outputImageScale), 24, RenderTextureFormat.ARGB32);
                outputTexture.filterMode = filterMode;
                imageSourceCamera.targetTexture = outputTexture;
                outputImage.texture = outputTexture;
            }

        }

        private void OnDisable()
        {
            currentHeight = 0;
            currentWidth = 0;
        }

        private bool HasSizeChanged()
        {
            if (currentHeight <= 0 || currentWidth <= 0)
            {
                return true;
            }

            var rectTransform = (RectTransform)transform;
            if (rectTransform.rect.width != currentWidth ||
                rectTransform.rect.height != currentHeight)
            {
                return true;
            }

            return false;
        }
    }
}
