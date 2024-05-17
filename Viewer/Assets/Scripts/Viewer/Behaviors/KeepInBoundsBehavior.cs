using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts.Viewer.Behaviors
{
    /// <summary>
    /// Keeps the game object within the given bounds
    /// </summary>
    public class KeepInBoundsBehavior: MonoBehaviour
    {
        [SerializeField]
        public Renderer boundsObj;

        [SerializeField]
        public float padding;

        private void Update()
        {
            Bounds bounds = new Bounds();
            if (this.boundsObj != null)
            {
                bounds = this.boundsObj.bounds;
            }

            if (bounds.size.x > 0 && bounds.size.y > 0 && bounds.size.z > 0)
            {
                Vector3 position = gameObject.transform.position;
                position.x = Mathf.Clamp(position.x, bounds.min.x, bounds.max.x);
                position.y = Mathf.Clamp(position.y, bounds.min.y, bounds.max.y);
                position.z = Mathf.Clamp(position.z, bounds.min.z, bounds.max.z);
                transform.position = position;
            }
        }
    }
}
