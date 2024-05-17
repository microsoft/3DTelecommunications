using Assets.Scripts.Viewer;
using UnityEngine;
using UnityEngine.UI;
using Assets.Scripts.Common.Extensions;

public class DrawToolOptionsManager : ViewerStateBehavior
{
    [SerializeField]
    private Slider sizeSlider;

    [SerializeField]
    private ColorSwatchesBehavior colorSelector;

    [SerializeField]
    private Button eraseButton;

    [SerializeField]
    private Button penButton;

    [SerializeField]
    private Button brushButton;

    [SerializeField]
    private Button textButton;

    [SerializeField]
    private Button arrowButton;

    [SerializeField]
    private Button defaultButton;

    [SerializeField]
    private Sprite _textIcon;

    [SerializeField]
    private AnnotationTool annotationTool;

    [SerializeField]
    private GameObject drawToolButton;

    [SerializeField]
    private GameObject drawToolOptionsContainer;

    [SerializeField]
    private GameObject sizeGroup;

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    protected override void OnEnable()
    {
        base.OnEnable();

        eraseButton?.onClick.AddListener(OnEraseButtonClick);
        penButton?.onClick.AddListener(OnPenButtonClick);
        brushButton?.onClick.AddListener(OnBrushButtonClick);
        textButton?.onClick.AddListener(OnTextButtonClick);
        arrowButton?.onClick.AddListener(OnArrowButtonClick);
        defaultButton?.onClick.AddListener(OnDefaultButtonClick);

        if (colorSelector != null)
        {
            colorSelector.Colors = SettingsManager.Instance.DrawToolColors;
            colorSelector.SelectedColor = annotationTool.brushColor;
            colorSelector.onSelectedColorChanged.AddListener(OnSelectedColorChanged);
        }

        if (sizeSlider != null)
        {
            sizeSlider.value = annotationTool.brushSize;
            sizeSlider.onValueChanged.AddListener(OnSizeSliderChanged);
        }

        annotationTool.TextIcon = _textIcon;

        UpdateToolButtons();
    }

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    protected override void OnDisable()
    {
        base.OnDisable();
        penButton?.onClick.RemoveListener(OnPenButtonClick);
        brushButton?.onClick.RemoveListener(OnBrushButtonClick);
        textButton?.onClick.RemoveListener(OnTextButtonClick);
        arrowButton?.onClick.RemoveListener(OnArrowButtonClick);
        colorSelector?.onSelectedColorChanged.RemoveListener(OnSelectedColorChanged);
        sizeSlider?.onValueChanged.RemoveListener(OnSizeSliderChanged);
        defaultButton?.onClick.RemoveListener(OnDefaultButtonClick);
    }

    /// <summary>
    /// Listener for when the text button is clicked on
    /// </summary>
    private void OnTextButtonClick()
    {
        annotationTool.SelectedDrawTool = DrawTool.Text;
        UpdateToolButtons();
    }

    /// <summary>
    /// Listener for when the arrow button is clicked on
    /// </summary>
    private void OnArrowButtonClick()
    {
        annotationTool.SelectedDrawTool = DrawTool.Arrow;
        UpdateToolButtons();
    }

    /// <summary>
    /// Listener for when the erase button is clicked on
    /// </summary>
    private void OnEraseButtonClick()
    {
        annotationTool.SelectedDrawTool = DrawTool.Erase;
        UpdateToolButtons();
    }

    /// <summary>
    /// Listener for when the pen button is clicked on
    /// </summary>
    private void OnPenButtonClick()
    {
        annotationTool.SelectedDrawTool = DrawTool.Pen;
        UpdateToolButtons();
    }

    /// <summary>
    /// Listener for when the brush button is clicked on
    /// </summary>
    private void OnBrushButtonClick()
    {
        annotationTool.SelectedDrawTool = DrawTool.Brush;
        UpdateToolButtons();
    }

    /// <summary>
    /// Listener for when the default draw button is clicked on
    /// </summary>
    private void OnDefaultButtonClick()
    {
        annotationTool.SelectedDrawTool = DrawTool.Default;
        UpdateToolButtons();
    }

    /// <summary>
    /// Listener for when a color swatch is clicked on
    /// </summary>
    private void OnSelectedColorChanged()
    {
        annotationTool.brushColor = colorSelector.SelectedColor;
        SettingsManager.Instance.DrawToolBrushColor = colorSelector.SelectedColor;
    }

    /// <summary>
    /// Listener for when the size slider has been changed
    /// </summary>
    /// <param name="value">The new value of the size slider</param>
    private void OnSizeSliderChanged(float value)
    {
        annotationTool.brushSize = Mathf.Clamp01(value);
        SettingsManager.Instance.DrawToolBrushSize = annotationTool.brushSize;
    }

    /// <summary>
    /// Updates the state of the tool buttons to match the current state of the system.
    /// </summary>
    private void UpdateToolButtons()
    {
        eraseButton?.SetSelected(annotationTool.SelectedDrawTool == DrawTool.Erase);
        penButton?.SetSelected(annotationTool.SelectedDrawTool == DrawTool.Pen);
        arrowButton?.SetSelected(annotationTool.SelectedDrawTool == DrawTool.Arrow);
        brushButton?.SetSelected(annotationTool.SelectedDrawTool == DrawTool.Brush);
        textButton?.SetSelected(annotationTool.SelectedDrawTool == DrawTool.Text);
        defaultButton?.SetSelected(annotationTool.SelectedDrawTool == DrawTool.Default);

        if (sizeGroup != null) 
        {
            // Can only change the size for the brush, pen, and erase tools
            sizeGroup.SetActive(
                annotationTool.SelectedDrawTool == DrawTool.Erase ||
                annotationTool.SelectedDrawTool == DrawTool.Pen ||
                annotationTool.SelectedDrawTool == DrawTool.Brush
            );
        }

        if (colorSelector.SelectedColor != annotationTool.brushColor)
        {
            colorSelector.SelectedColor = annotationTool.brushColor;
        }

        if (sizeSlider.value != annotationTool.brushSize)
        {
            sizeSlider.SetValueWithoutNotify(annotationTool.brushSize);
        }
    }

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    private void Update()
    {
        bool isActive = annotationTool.isActiveAndEnabled && drawToolButton.activeInHierarchy;
        drawToolOptionsContainer.SetActive(isActive);

        if (isActive)
        {
            UpdateToolButtons();
        }
    }
}
