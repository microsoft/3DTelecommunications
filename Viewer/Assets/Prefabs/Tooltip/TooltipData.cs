using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Prefabs.Tooltip
{
    [Serializable]
    public struct TooltipData
    {
        public static TooltipData Empty = new TooltipData() { Contents = null, Title = null };

        public string Title;

        [TextArea]
        public string Contents;
    }
}
