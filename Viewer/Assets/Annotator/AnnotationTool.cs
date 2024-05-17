using Assets.Scripts.Common;
using Assets.Scripts.Viewer.Behaviors;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using System.Collections;
using Assets.Scripts.Viewer;
using Assets.Scripts.Viewer.Common;
using UnityEngine.EventSystems;
using UnityEngine.UIElements;

public enum DrawTool
{
    Pen,
    Brush,
    Erase,
    Text,
    Arrow,
    Default
}

public class AnnotationTool : MonoBehaviour 
{
    public Material TexturePaintMaterial;

    [Header("Settings")]
    public Color brushColor;
    public float brushOpacity;
    public float brushSize;
    public float brushHardness;

    [Header("Instanced")]
    public RenderTexture paintedTexture;
    
    private RenderTexture tempTexture;
    
    private CommandBuffer paintTextureCommandBuffer;
    private Material targetMaterial;
    private Mesh targetMesh;

    public bool drawAlways = false;

    private Camera targetCamera;

    [Header("Text/Arrow tools")]
    [SerializeField]
    private RectTransform placementArea;

    [SerializeField]
    private GameObject textPrefab;

    [SerializeField]
    private GameObject arrowPrefab;

    private DrawTool __selectedDrawTool = DrawTool.Brush;

    private List<TextAnnotationBehavior> _textObjs = new List<TextAnnotationBehavior>();
    private List<ArrowAnnotationBehavior> _arrowObjs = new List<ArrowAnnotationBehavior>();

    private Texture2D _cursorTexture;

    [SerializeField]
    private Sprite _textIcon;


    private CursorManager.CursorInfo _textCursor;

    private ArrowAnnotationBehavior _currentArrow;
    private TextAnnotationBehavior _currentText;
    private GameObject _lastMouseUpSelectedGameObject;

    /// <summary>
    /// Gets/sets the selected draw tool
    /// </summary>
    public DrawTool SelectedDrawTool
    {
        get
        {
            return __selectedDrawTool;
        }
        set
        {
            if (__selectedDrawTool != value)
            {
                __selectedDrawTool = value;
                ApplyToolCursor(value);
                ApplyDrawOptionsToMaterial();
            }
        }
    }

    /// <summary>
    /// Gets/sets the icon to use for the text tool cursor
    /// </summary>
    public Sprite TextIcon
    {
        get
        {
            return _textIcon;
        }
        set
        {    
            _textIcon = value;
            CreateToolCursor();
        }
    }

    /// <summary>
    /// Updates the target mesh and material
    /// </summary>
    /// <param name="mesh">The mesh the tool is drawing on</param>
    /// <param name="mat">The material the tool is drawing on</param>
    public void UpdateTarget(Mesh mesh, Material mat)
    {
        if (mat.shader.name != "Unlit/AnnotatedTexture")
        {
            Debug.LogError("Annotation Tool only works with the Unlit/AnnotationTexture shader");
        }
        else
        {
            targetMaterial = mat;
            targetMesh = mesh;
            RefreshTextures();
        }
    }

    /// <summary>
    /// Updates the position of the mouse, and the object that was hit from raycasting
    /// </summary>
    /// <param name="validInputRect">The area that is valid for the mouse to be in</param>
    /// <param name="screenPos">The actual screen position of the mouse</param>
    /// <param name="hitInfo">The hit based on raycasting</param>
    public void UpdateMousePosition(RectTransform validInputRect, Vector3? screenPos, RaycastHit? hitInfo, bool mouseCaptured, bool mouseDown, bool mouseUp)
    {
        bool active = false;
        if (!mouseUp && (SelectedDrawTool == DrawTool.Pen || SelectedDrawTool == DrawTool.Brush || SelectedDrawTool == DrawTool.Erase))
        {  
            if (hitInfo != null && mouseDown && mouseCaptured)
            {
                Vector4 mousePositionOnObject = hitInfo.Value.point;
                //non zero vector means we actually hit something
                if (mousePositionOnObject.sqrMagnitude > 0)
                {
                    TexturePaintMaterial.SetMatrix("mesh_Object2World", hitInfo.Value.collider.gameObject.transform.localToWorldMatrix);
                    TexturePaintMaterial.SetVector("_Mouse", mousePositionOnObject);
                    active = true;
                }
            }
        } 
        else if (screenPos != null)
        {
            bool canPlace = mouseCaptured && mouseDown;
            if (SelectedDrawTool == DrawTool.Text)
            {
                // If we didn't just click off of a text annotation
                if (canPlace && !IsLastSelectedObjectATextAnnotation())
                {
                    if (_currentText == null)
                    {
                        GameObject textObj = Instantiate(textPrefab, placementArea.transform);
                        textObj.transform.position = screenPos.Value;
                        _currentText = textObj.GetComponent<TextAnnotationBehavior>();
                        _currentText.SelectedColor = brushColor;
                        _currentText.name = $"annotation-{_textObjs.Count + 1}";
                        _currentText.Destroyed += UpdateTextObjs;
                        ResizePanel resize = textObj.GetComponent<ResizePanel>();
                        if (resize != null && validInputRect != null)
                        {
                            resize.validRect = validInputRect;
                        }
                        _textObjs.Add(_currentText);
                        _currentText.Focus();
                    } 
                } 
                else
                {
                    _currentText = null;
                }
            }
            else if (SelectedDrawTool == DrawTool.Arrow)
            {
                if (canPlace)
                {
                    if (_currentArrow == null)
                    {
                        GameObject newObj = Instantiate(arrowPrefab, placementArea.transform);
                        _currentArrow = newObj.GetComponent<ArrowAnnotationBehavior>();
                        _currentArrow.transform.position = screenPos.Value;
                        ((RectTransform)_currentArrow.transform).sizeDelta =
                           new Vector2(
                               0f,
                               ((RectTransform)_currentArrow.transform).sizeDelta.y
                           );
                        _currentArrow.GetComponent<ArrowAnnotationBehavior>().ArrowColor = SettingsManager.Instance.DrawToolBrushColor;
                        ResizePanel resize = newObj.GetComponent<ResizePanel>();
                        if (resize != null && validInputRect != null)
                        {
                            resize.enabled = false;
                            resize.validRect = validInputRect;
                        }
                        _arrowObjs.Add(_currentArrow);
                    } 
                    else
                    {
                        var delta = screenPos.Value - _currentArrow.transform.position;
                        ((RectTransform)_currentArrow.transform).sizeDelta =
                            new Vector2(
                                delta.magnitude / 1.5f,
                                ((RectTransform)_currentArrow.transform).sizeDelta.y
                            );
                        Quaternion rot = Quaternion.LookRotation(_currentArrow.transform.forward, delta);
                        _currentArrow.transform.localRotation = rot * Quaternion.AngleAxis(90, _currentArrow.transform.forward);
                    }
                } 
                else
                {
                    if (_currentArrow != null)
                    {
                        ResizePanel panel = _currentArrow.GetComponent<ResizePanel>();
                        if (panel != null)
                        {
                            panel.enabled = true;
                        }
                    }
                    _currentArrow = null;
                }
            }
        }

        TexturePaintMaterial.SetFloat("_Active", active ? 1.0f : 0.0f);
    }    

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    private void OnEnable()
    {
        if (targetCamera == null)
        {
            targetCamera = Camera.main;
        }

        //create texture we are painting onto
        paintedTexture = new RenderTexture(4096, 4096, 0, RenderTextureFormat.BGRA32);
        paintedTexture.useMipMap = false;
        paintedTexture.filterMode = FilterMode.Point;
        paintedTexture.enableRandomWrite = true;

        tempTexture = new RenderTexture(4096, 4096, 0, RenderTextureFormat.BGRA32);
        tempTexture.useMipMap = false;
        tempTexture.filterMode = FilterMode.Point;
        tempTexture.enableRandomWrite = true;

        //load from settings if we can
        if (SettingsManager.Instance != null)
        {
            SettingsManager settings = SettingsManager.Instance;
            brushColor = settings.DrawToolBrushColor;
            brushOpacity = settings.DrawToolBrushOpacity;
            brushSize = settings.DrawToolBrushSize;
            brushHardness = settings.DrawToolBrushHardness;
            settings.OnChange += OnSettingsManagerSettingsChanged;
        }

        // Feed the currently painted texture to our painting shader
        TexturePaintMaterial.SetTexture("_MainTex", paintedTexture);
        ApplyDrawOptionsToMaterial();
        RefreshTextures();
        ApplyToolCursor(SelectedDrawTool);
    }

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    private void OnDisable()
    {
        paintedTexture.Release();
        tempTexture.Release();
        paintedTexture = null;
        tempTexture = null;
        if (targetMaterial != null)
        {
            TexturePaintMaterial.SetMatrix("mesh_Object2World", Matrix4x4.identity);
            TexturePaintMaterial.SetVector("_Mouse", Vector4.zero);

            targetMaterial.SetTexture("_AnnotatedTexture", null);
        }

        if (paintTextureCommandBuffer != null)
        {
            targetCamera.RemoveCommandBuffer(CameraEvent.AfterEverything, paintTextureCommandBuffer);
        }

        if (SettingsManager.Instance != null)
        {
            SettingsManager.Instance.OnChange -= OnSettingsManagerSettingsChanged;
        }

        // Remove it if we have it applied
        CursorManager.RemoveCursor(_textCursor);

        foreach (ArrowAnnotationBehavior obj in _arrowObjs)
        {
            if (obj != null)
            {
                obj.Remove();
            }
        }
        _arrowObjs = new List<ArrowAnnotationBehavior>();

        foreach (TextAnnotationBehavior obj in _textObjs)
        {
            if (obj != null)
            {
                obj.Remove();
            }
        }
        _textObjs = new List<TextAnnotationBehavior>();
    }

    /// <summary>
    /// Refreshes the textures used by the target material
    /// </summary>
    private void RefreshTextures()
    {
        if (enabled)
        {
            // Update
            targetMaterial.SetTexture("_AnnotatedTexture", paintedTexture);

            if (paintTextureCommandBuffer != null)
            {
                targetCamera.RemoveCommandBuffer(CameraEvent.AfterEverything, paintTextureCommandBuffer);
            }

            // Draw onto the tempTexture using the painting shader (TexturePaintMaterial), then copy those pixels back
            // to the main painted texture
            paintTextureCommandBuffer = new CommandBuffer();
            paintTextureCommandBuffer.name = "PaintTexture";
            paintTextureCommandBuffer.SetRenderTarget(tempTexture);
            paintTextureCommandBuffer.DrawMesh(targetMesh, Matrix4x4.identity, TexturePaintMaterial);
            paintTextureCommandBuffer.Blit(tempTexture, paintedTexture);

            // add it to after we render everything else
            targetCamera.AddCommandBuffer(CameraEvent.AfterEverything, paintTextureCommandBuffer);
        }
    }

    /// <summary>
    /// Listener for when the settings manager has a setting change
    /// </summary>
    /// <param name="setting">The setting that was changed</param>
    /// <param name="value">The new value of the setting</param>
    private void OnSettingsManagerSettingsChanged(SettingName setting, object value)
    {
        SettingsManager settings = SettingsManager.Instance;
        if (setting == settings.DrawToolBrushColorKey)
        {
            brushColor = settings.DrawToolBrushColor;
        }
        if (setting == settings.DrawToolBrushOpacityKey)
        {
            brushOpacity = settings.DrawToolBrushOpacity;
        }
        if (setting == settings.DrawToolBrushHardnessKey)
        {
            brushHardness = settings.DrawToolBrushHardness;
        }
        if (setting == settings.DrawToolBrushSizeKey)
        {
            brushSize = settings.DrawToolBrushSize;
        }
        ApplyDrawOptionsToMaterial();
    }

    /// <summary>
    /// Applies all the current drawing options to the drawing material
    /// </summary>
    private void ApplyDrawOptionsToMaterial()
    {
        TexturePaintMaterial.SetColor("_BrushColor", SelectedDrawTool == DrawTool.Erase ? Color.clear : brushColor);
        TexturePaintMaterial.SetFloat("_BrushOpacity", brushOpacity);
        TexturePaintMaterial.SetFloat("_BrushSize", brushSize);
        TexturePaintMaterial.SetFloat(
            "_BrushHardness",
            SelectedDrawTool == DrawTool.Pen ? 1.0f : brushHardness
        );
    }
    
    /// <summary>
    /// Creates the tool cursor
    /// </summary>
    private void CreateToolCursor()
    {
        _textCursor = new CursorManager.CursorInfo(
            Utils.MakeTexture2DFromSprite(_textIcon),
            Utils.GetHotspotFromSprite(_textIcon)
        );
    }

    /// <summary>
    /// Applies the current tool's cursor to the screen
    /// </summary>
    /// <param name="tool"></param>
    private void ApplyToolCursor(DrawTool tool)
    {
        CursorManager.RemoveCursor(_textCursor);
        if (tool == DrawTool.Text)
        {
            CursorManager.AddCursor(_textCursor);
        }
    }

    /// <summary>
    /// Updates the list of text objects on the screen
    /// </summary>
    /// <param name="labelToRemove">The label of the text annotation to remove</param>
    private void UpdateTextObjs(string labelToRemove)
    {
        List<TextAnnotationBehavior> newList = new List<TextAnnotationBehavior>();
        int index = 0;
        foreach (TextAnnotationBehavior obj in _textObjs)
        {
            if (obj.name != labelToRemove)
            {
                obj.name = $"annotation-{index}";
                newList.Add(obj);
                index = index + 1;
            }
        }
        _textObjs = newList;
    }

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    private void Update()
    {
        if (Input.GetMouseButtonUp((int)MouseButton.LeftMouse))
        {
            _lastMouseUpSelectedGameObject = EventSystem.current.currentSelectedGameObject;
        }
    }

    /// <summary>
    /// Returns true if the last selected object was a text annotation
    /// </summary>
    /// <returns></returns>
    private bool IsLastSelectedObjectATextAnnotation()
    {
        return 
            _lastMouseUpSelectedGameObject != null &&
            _lastMouseUpSelectedGameObject.GetComponentInParent<TextAnnotationBehavior>() != null;
    }
}
