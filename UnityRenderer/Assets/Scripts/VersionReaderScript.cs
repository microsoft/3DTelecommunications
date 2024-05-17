using UnityEngine;
using UnityEngine.UI;

public class VersionReaderScript : MonoBehaviour
{
    void Start()
    {
        Text t = GetComponent<Text>( );

        t.text = "ver: ?.?.?";
        if( Application.version != null )
        {
            if( Application.version != "" )
            {
                t.text = "ver:" + Application.version;
            }
        }
    }
}
