using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.Scripts.Viewer.Models
{
    /// <summary>
    /// A binding between a view and a dimension
    /// </summary>
    public struct ViewDimension
    {
        public int ViewId;
        public Dimension Dimension;

        public ViewDimension(int viewId, Dimension dimension)
        {
            ViewId = viewId;
            Dimension = dimension;
        }

        public ViewDimension Clone()
        {
            return new ViewDimension()
            {
                ViewId = ViewId,
                Dimension = Dimension,
            };
        }

        public override string ToString()
        {
            return $"{ViewId}: {Dimension}";
        }
    }
}
