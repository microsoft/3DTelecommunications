using UnityEngine;
using UnityEngine.UI;

[ExecuteAlways]
public class ColorSwatchBehavior : Button
{
    [SerializeField]
    private Color color;

    [SerializeField]
    private Image output;

    [SerializeField]
    private int selectionBorderSize = 2;

    [SerializeField]
    private bool selected = true;

    /// <summary>
    /// If the swatch is selected
    /// </summary>
    public bool Selected
    {
        get
        {
            return selected;
        }
        set
        {
            selected = value;
            UpdateSelectionState(value);
        }
    }

    /// <summary>
    /// The color of the swatch
    /// </summary>
    public Color Color
    {
        get 
        { 
            return color; 
        }
        set
        {
            color = value;
            output.color = value;
        }
    }

    #if UNITY_EDITOR

    /// <summary>
    /// Lifecyle function that gets called when the editor changes values
    /// </summary>
    protected override void OnValidate()
    {
        base.OnValidate();

        // Lil jank, but causes the logic to re-run when the editor inspector changes values
        Color = Color;
        Selected = Selected;
    }

    #endif

    /// <summary>
    /// Lifecycle function that gets called when the behavior is enabled
    /// </summary>
    protected override void OnEnable()
    {
        base.OnEnable();

        if (output == null)
        {
            output = GetComponent<Image>();
        }
        UpdateSelectionState(Selected);
    }

    /// <summary>
    /// Updates the UI to match the selection state.
    /// </summary>
    /// <param name="selected"></param>
    private void UpdateSelectionState(bool selected)
    {
        RectTransform rect = (RectTransform)output.transform;
        if (selected)
        {
            rect.offsetMin = new Vector2(selectionBorderSize, selectionBorderSize);
            rect.offsetMax = new Vector2(-selectionBorderSize, -selectionBorderSize);
        } 
        else
        {
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;
        }
    }
}
