using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Assets.Scripts.Viewer.Common
{
    public struct Toast
    {
        public string Title;
        public string Contents;
        public Action Callback;
    }
}
