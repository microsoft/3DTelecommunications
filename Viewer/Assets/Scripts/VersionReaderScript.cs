using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using UnityEngine.UI;

public class VersionReaderScript : MonoBehaviour
{
    // Start is called before the first frame update
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
