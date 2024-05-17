using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Assets.Scripts.Viewer.Behaviors
{
    public class ImageAspectRatioFitter : AspectRatioFitter
    {
        // IMPORTANT - Check this property is referenced in the editor, if you change it, update it there too.
        [SerializeField]
        private MaskableGraphic toSize;

        protected override void OnEnable()
        {
            base.OnEnable();
            if (toSize == null)
            {
                toSize = GetComponent<RawImage>();
                if (toSize == null)
                {
                    toSize = GetComponent<Image>();
                }
            }
        }

        private void FixedUpdate()
        {
            if (toSize != null && toSize.mainTexture != null)
            {
                aspectRatio = (float)toSize.mainTexture.width / (float)toSize.mainTexture.height;
            }
        }
    }
}
