using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts.Common
{
    public delegate void TextureUpdate();

    public interface ITextureSource
    {
        string Name { get; }

        Vector2Int Dimensions { get; }

        Texture Frame { get; }

        event TextureUpdate OnTextureUpdate;
    }
}
