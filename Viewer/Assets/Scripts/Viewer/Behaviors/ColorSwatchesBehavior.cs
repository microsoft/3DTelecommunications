using UnityEngine;
using System.Linq;
using UnityEngine.UI;
using UnityEngine.Events;

public class ColorSwatchesBehavior : MonoBehaviour
{
    [SerializeField]
    private Color[] colors = new Color[] {
        Color.red,
        Color.white,
        Color.blue
    };

    [SerializeField]
    private GameObject swatchPrefab;

    /// <summary>
    /// THe list of instantiated swatches
    /// </summary>
    private ColorSwatchBehavior[] swatches;

    /// <summary>
    /// Event for when the selected color is changed
    /// </summary>
    public UnityEvent onSelectedColorChanged;

    /// <summary>
    /// Gets or sets the swatch colors
    /// </summary>
    public Color[] Colors
    {
        get
        {
            return colors;
        }
        set
        {
            colors = value;

            SyncSwatches();
        }
    }

    /// <summary>
    /// Gets/sets the selected color
    /// </summary>
    public Color SelectedColor
    {
        get
        {
            if (swatches != null)
            {
                ColorSwatchBehavior selected = swatches.FirstOrDefault(n => n.Selected);
                if (selected != null)
                {
                    return selected.Color;
                }
                return swatches[0].Color;
            }
            return Color.clear;
        }
        set
        {
            if (swatches != null)
            {
                bool anySelected = false;
                foreach (ColorSwatchBehavior swatch in swatches)
                {
                    // Are the colors close enough, if so mark them as "selected"
                    swatch.Selected = ((Vector4)swatch.Color - (Vector4)value).magnitude < 0.01;
                    anySelected = anySelected || swatch.Selected;
                }

                // Select the first if none are selected
                if (!anySelected)
                {
                    swatches[0].Selected = true;
                }
            }

            onSelectedColorChanged?.Invoke();
        }
    }

    /// <summary>
    /// OnDisable lifecycle method
    /// </summary>
    private void OnDisable()
    {
        DestroySwatches(swatches);
        swatches = null;
    }

    /// <summary>
    /// OnEnabled lifecycle method
    /// </summary>
    private void OnEnable()
    {
        Color oldSel = SelectedColor;
        SyncSwatches();
        SelectedColor = oldSel;
        //var top = gameObject.GetComponentsInParent<VerticalLayoutGroup>().Last();
        //LayoutRebuilder.ForceRebuildLayoutImmediate(
        //    top.transform as RectTransform
        //);
        //LayoutRebuilder.ForceRebuildLayoutImmediate(transform.root as RectTransform);
    }

    /// <summary>
    /// Syncronizes the list of swatches we have with the list of colors
    /// </summary>
    private void SyncSwatches()
    {
        DestroySwatches(swatches);
        swatches = InstatiateSwatches(colors, swatchPrefab);
        foreach (ColorSwatchBehavior swatch in swatches)
        {
            swatch.onClick.AddListener(() => this.OnSwatchClick(swatch));
            swatch.transform.SetParent(transform, false);
        }
    }

    /// <summary>
    /// Event listener for when a swatch is clicked on
    /// </summary>
    private void OnSwatchClick(ColorSwatchBehavior swatch)
    {
        SelectedColor = swatch.Color;
    }

    /// <summary>
    /// Destroys the list of given game objects
    /// </summary>
    /// <param name="swatches"></param>
    private static void DestroySwatches(ColorSwatchBehavior[] swatches, bool immediate = false)
    {
        if (swatches != null)
        {
            foreach (ColorSwatchBehavior swatch in swatches)
            {
                if (swatch != null && swatch.gameObject)
                {
                    if (immediate)
                    {
                        DestroyImmediate(swatch.gameObject);
                    }
                    else
                    {
                        Destroy(swatch.gameObject);
                    }
                }
            }
        }
    }

    /// <summary>
    /// Instantiates swatches for the given colors
    /// </summary>
    /// <param name="colors">The colors for the satches</param>
    /// <param name="prefab">The swatch prefab</param>
    /// <returns></returns>
    private static ColorSwatchBehavior[] InstatiateSwatches(Color[] colors, GameObject prefab)
    {
        return colors
            .Select(n => {
                GameObject obj = Instantiate(prefab);
                LayoutElement le = obj.AddComponent<LayoutElement>();

                le.minWidth = 20;
                le.minHeight = 20;
                le.preferredWidth = 20;
                le.preferredHeight = 20;

                ColorSwatchBehavior swatch = obj.GetComponent<ColorSwatchBehavior>();
                swatch.Color = n;
                return swatch;
            })
            .ToArray();
    }
}

