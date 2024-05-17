using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

[Serializable]
public class Configuration : ScriptableObject
{
    public int NumPods;
    public int ColorWidth;
    public int ColorHeight;
    public int ColorPixelBytes = 1;
    public string NetworkHostIP;
    public string NetworkHostPort;
    public TextureFormat TextureFormat = TextureFormat.Alpha8;

    public string ToString()
    {
        return $"Configuration:\nNumPods: {NumPods}\nColorWidth: {ColorWidth}\nColorHeight: {ColorHeight}\nColorPixelBytes: {ColorPixelBytes}\nNetworkHost: {NetworkHostIP}:{NetworkHostPort}\nTexture Format: {TextureFormat}";
    }
}

