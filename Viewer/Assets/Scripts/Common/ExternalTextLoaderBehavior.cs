using System.IO;
using TMPro;
using UnityEngine;

namespace Assets.Scripts.Common
{
    public class ExternalTextLoaderBehavior: MonoBehaviour
    {
        [SerializeField]
        public TextMeshProUGUI textContainer;

        [SerializeField]
        public string streamingFileTextPath;

        private void OnEnable()
        {
            if (textContainer != null)
            {
                string path = Path.Combine(Application.streamingAssetsPath, "./", streamingFileTextPath);
                if (!File.Exists(path))
                {
                    OutputHelper.OutputLog($"Could not find streaming asset for text: {streamingFileTextPath}");
                }
                else
                {
                    textContainer.text = File.ReadAllText(path);
                }
            }
        }
    }
}
