using System;
using UnityEngine;

namespace Assets.Scripts.Common
{
    public interface ITransformable
    {
        Quaternion Rotation { get; set; }

        Vector3 Position { get; set; }

        Vector3 Scale { get; set; }

        event Action TransformUpdated;
    }
}
