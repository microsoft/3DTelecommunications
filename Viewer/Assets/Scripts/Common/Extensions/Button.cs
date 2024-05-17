using UnityEngine;
using UnityEngine.UI;

namespace Assets.Scripts.Common.Extensions
{
    public static class ButtonExtensions
    {
        public static void SetSelected(this Button obj, bool selected) { 
            if (selected)
            {
                obj.OnSelect(null);
            } 
            else
            {
                obj.OnDeselect(null);
            }
        }
    }
}
