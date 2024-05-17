using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// A simple FPS display to measure the Montage4D algorithm
/// You can also use Stats button to see the real-time FPS in Unity
/// </summary>
public class FPSDisplay : MonoBehaviour
{
    float deltaTime = 0.0f;
    List<float> fpsList = new List<float>();
    void Update()
    {
        deltaTime += (Time.deltaTime - deltaTime) * 0.1f;
        fpsList.Add(1.0f / deltaTime);
    }

    void writeToCSV()
    {

    }

    void OnGUI()
    {
        int w = Screen.width, h = Screen.height;

        GUIStyle style = new GUIStyle();

        Rect rect = new Rect(0, 0, w, h * 2 / 100);
        style.alignment = TextAnchor.UpperLeft;
        style.fontSize = h * 2 / 100;
        style.normal.textColor = new Color(0.0f, 0.0f, 0.5f, 1.0f);
        float msec = deltaTime * 1000.0f;
        float fps = 1.0f / deltaTime;
        string text = string.Format("{0:0.0} ms ({1:0.} fps)", msec, fps);
        GUI.Label(rect, text, style);
    }
}