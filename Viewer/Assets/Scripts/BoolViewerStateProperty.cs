using Assets.Scripts.Common;
using Assets.Scripts.Viewer;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts
{
    [Serializable]
    public class BoolViewerStateProperty
    {
        public string name;

        public ObservableProperty<bool> Property
        {
            get
            {
                return ViewerManager.Current().Store.GetState().GetPropertyByName(name) as ObservableProperty<bool>;
            }
        }
    }
}
