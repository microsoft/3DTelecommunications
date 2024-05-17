using UnityEngine;

namespace Assets.Scripts.Common.Extensions
{
    public static class Texture2DExtensions
    {
        /// <summary>
        /// Applies a tint to the given texture.  
        /// </summary>
        /// <param name="texture"></param>
        /// <param name="tint"></param>
        /// <returns></returns>
        public static Texture2D ApplyTint(this Texture2D texture, Color tint)
        {
            int width = texture.width;
            int height = texture.height;
            Texture2D tinted = new Texture2D(
                width,
                height,
                texture.format,
                false
            );

            Color[] colors = texture.GetPixels();
            for (int i = 0; i < colors.Length; i++)
            {
                colors[i] = colors[i] * tint;
            }
            tinted.SetPixels(colors);

            return tinted;
        }
        public static Texture2D Crop(this Texture2D texture, int offsetX, int offsetY, int width, int height, bool flipY)
        {
            Texture2D cropped = new Texture2D(
                width,
                height,
                texture.format,
                false
            );

            //Height of image in pixels
            for (int y = 0; y < height; y++)
            {
                //Width of image in pixels
                for (int x = 0; x < width; x++)
                {
                    int sy = offsetY + y;
                    int sx = offsetX + x;
                    Color cPixelColour = texture.GetPixel(sx, flipY ? sy : texture.height - sy);
                    cropped.SetPixel(x, flipY ? height - y : y, cPixelColour);
                }
            }

            return cropped;
        }
    }
}
