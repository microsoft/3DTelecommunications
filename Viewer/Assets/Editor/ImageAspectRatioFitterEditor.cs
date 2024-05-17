using Assets.Scripts.Viewer.Behaviors;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(ImageAspectRatioFitter))]
public class ImageAspectRatioFitterEditor : UnityEditor.UI.AspectRatioFitterEditor
{
    SerializedProperty toSizeProp;

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();//Draw inspector UI of ImageEditor
        
        EditorGUILayout.PropertyField(toSizeProp);

        serializedObject.ApplyModifiedProperties();
    }

    protected override void OnEnable()
    {
        base.OnEnable();

        // Fetch the objects from the GameObject script to display in the inspector
        toSizeProp = serializedObject.FindProperty("toSize");
    }
}