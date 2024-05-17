using Assets.Scripts;
using Assets.Scripts.Viewer.State;
using System;
using System.Linq;
using UnityEditor;
using UnityEngine;

[CustomPropertyDrawer(typeof(BoolViewerStateProperty))]
public class ViewerStateBoolPropertyEditor : PropertyDrawer
{
    // Draw the property inside the given rect
    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {
        EditorGUI.BeginProperty(position, label, property);

        //// Draw label
        position = EditorGUI.PrefixLabel(position, GUIUtility.GetControlID(FocusType.Passive), label);
        SerializedProperty name = property.FindPropertyRelative("name");

        string[] propertyNames = ViewerState.GetBoolPropertyNames().ToArray();

        string value = string.IsNullOrEmpty(name.stringValue) ? propertyNames[0] : name.stringValue;
        int selectedId = Mathf.Clamp(EditorGUI.Popup(position, Array.IndexOf(propertyNames, value), propertyNames), 0, propertyNames.Length);

        EditorGUI.indentLevel = 0;

        name.stringValue = propertyNames[selectedId];

        EditorGUI.EndProperty();
    }
}