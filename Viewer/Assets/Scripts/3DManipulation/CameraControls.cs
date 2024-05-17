using Assets.Prefabs.Tooltip;
using Assets.Scripts.Common.Extensions;
using Assets.Scripts.Events;
using Assets.Scripts.Viewer;
using Assets.Scripts.Viewer.Common;
using Assets.Scripts.Viewer.Models;
using HoloportationDecoders.Holoportation;
using System;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Rendering;
using UnityEngine.UIElements;

public class CameraControls : ViewerStateBehavior, IPointerDownHandler, IPointerEnterHandler
{
    // what we use as rotational pivot
    public Transform focusTarget;
    public Camera targetCamera;
    public Camera templateCamera;
    public GameObject focusReticle;

    // area in which we can control the camera
    public RectTransform validInputRectBound;

    private float camFOV;
    private Vector3 mouseDownPosition;
    private Vector3 lastMousePosition;
    private Vector3 camStartPosition;
    private Vector3 panPoint;
    private Quaternion camStartRotation;
    private ViewerTool activeViewerTool = ViewerTool.None;
    private static TooltipData FocusTooltip = new TooltipData()
    {
        Contents = "Click to set a focus point."
    };
    private bool userMovedReticle = false;

    bool isControlDown = false;
    bool isMouseOver = false;

    [Header("AnnotationTool")]
    public AnnotationTool annotationToolScript;
    public UnityEngine.UI.Button drawingToolUIButton;
    public UnityEngine.UI.Button navigationOverrideToggleButton;

    [SerializeField]
    private bool _reticleMode = false;

    public BoolEvent OnAnnotationModeChanged;
    public BoolEvent OnReticleModeChanged;

    private bool mouseCaptured;

    public bool ReticleMode
    {
        get { return _reticleMode; }
        set
        {
            if (_reticleMode != value)
            {
                _reticleMode = value;
                userMovedReticle = false;
                if (focusReticle != null)
                {
                    if (focusTarget != null)
                    {
                        focusReticle.transform.position = focusTarget.position;
                    }

                    SizeAndPositionReticle();
                    UpdateViewTooltip();
                }


                // Turn off the reticle immediately
                if (!_reticleMode)
                {
                    focusReticle.gameObject.SetActive(false);
                }

                OnReticleModeChanged?.Invoke(value);
            }
        }
    }

    public bool InAnnotationMode
    {
        get { return annotationToolScript.enabled; }
        set
        {
            annotationToolScript.enabled = value;
            if (value)
            {
                ComputeCollision();
            }
            OnAnnotationModeChanged?.Invoke(value);
        }
    }
    
    /// <summary>
    /// True if the user is in navigation override mode,
    /// where the draw tool is temporarily disabled to allow full navigation
    /// </summary>
    public bool InNavigationOverrideMode
    {
        get;
        set;
    } = false;

    // Start is called before the first frame update
    protected override void OnEnable()
    {
        RegisterStateChangeHandler(n => n.ActiveTool, OnActiveToolChanged);
        RegisterStateChangeHandler(n => n.Paused, OnViewerPausedChanged);

        base.OnEnable();

        lastMousePosition = Vector3.zero;

        if (templateCamera != null)
        {
            camStartPosition = templateCamera.transform.position;
            camStartRotation = templateCamera.transform.rotation;
            camFOV = templateCamera.fieldOfView;

            targetCamera.transform.parent = templateCamera.transform.parent;

            //targetCamera.transform.localPosition = targetCamera.transform.parent.position * -1;
            //targetCamera.transform.localRotation = Quaternion.identity;

            ResetCamera();
        }
        else
        {
            camStartPosition = targetCamera.transform.position;
            camStartRotation = targetCamera.transform.rotation;
            camFOV = targetCamera.fieldOfView;
        }

        if (focusReticle != null && focusTarget != null) {
            focusReticle.gameObject.SetActive(ReticleMode);
            focusReticle.transform.position = focusTarget.position;
        }

        OnViewerPausedChanged(GetStateProperty(n => n.Paused).Value, false);
        UpdateAnnotationToolRenderTarget();

        HoloportationDecoders.Color2D.HoloportRawFrameViewer hqViewer = ViewerManager.Current().hqHoloportViewer;
        hqViewer.FrameDecoded += OnHQViewerFrameUpdated;

        if (navigationOverrideToggleButton) {
            navigationOverrideToggleButton.onClick.AddListener(OnNavigationOverrideClicked);
        }
    }

    protected override void OnDisable()
    {
        base.OnDisable();

        HoloportationDecoders.Color2D.HoloportRawFrameViewer hqViewer = ViewerManager.Current().hqHoloportViewer;
        hqViewer.FrameDecoded -= OnHQViewerFrameUpdated;

        InAnnotationMode = false;

        if (navigationOverrideToggleButton)
        {
            navigationOverrideToggleButton.onClick.RemoveListener(OnNavigationOverrideClicked);
        }
    }

    private void OnNavigationOverrideClicked()
    {
        InNavigationOverrideMode = !InNavigationOverrideMode;
    }

    private void OnHQViewerFrameUpdated(SubMeshComponents arg0)
    {
        UpdateAnnotationToolRenderTarget();
    }

    private void OnViewerPausedChanged(bool paused, bool prevPaused)
    {
        drawingToolUIButton.gameObject.SetActive(paused);

        if (!paused) {
            InAnnotationMode = false;
        }
    }

    private void UpdateAnnotationToolRenderTarget()
    {
        if (annotationToolScript != null)
        {
            (Material material, Mesh mesh) = GetActiveMaterialAndMesh();
            annotationToolScript.UpdateTarget(mesh, material);
        }
    }

    public void OnValidate()
    {
        if (_reticleMode != focusReticle.activeSelf)
        {
            ReticleMode = _reticleMode;
        }
    }

    public void ResetCamera()
    {
        targetCamera.transform.position = camStartPosition;
        targetCamera.transform.rotation = camStartRotation;
        targetCamera.fieldOfView = camFOV;

        ReticleMode = false;

        LookAtFocusTarget();
    }

    public void LookAtFocusTarget()
    {
        Vector3 lookAtPoint = focusTarget.position;
        if (ReticleMode && focusReticle != null && focusReticle.activeSelf)
        {
            lookAtPoint = focusReticle.transform.position;
        } 
        else
        {
            Bounds? b = focusTarget.gameObject.ComputeBounds();
            if (b != null)
            {
                // Try to look at center mass
                lookAtPoint = b.Value.center;
            }
        }
        
        targetCamera.transform.LookAt(lookAtPoint);

        Debug.DrawLine(targetCamera.transform.position, lookAtPoint, Color.white, 3);
    }

    public void ZoomToFit()
    {
        targetCamera.fieldOfView = camFOV;
        targetCamera.FitIntoView(focusTarget.gameObject);
    }
    public void ZoomToLifeSize()
    {
        targetCamera.FitToLifesizeFOV(focusTarget.gameObject);
    }

    void Update()
    {
        drawingToolUIButton?.SetSelected(InAnnotationMode);
        navigationOverrideToggleButton?.SetSelected(InNavigationOverrideMode);

        Vector2 mouseInRectPos = validInputRectBound.InverseTransformPoint(Input.mousePosition);
        //dont update if that bound is not active in UI hierarchy OR mouse is not withint bounds
        isMouseOver = (transform as RectTransform).rect.Contains(mouseInRectPos);

        if (isMouseOver)
        {
            UpdateKeyboardFlags();
            UpdateMouseFlags();

            bool userActivelyAnnotating = IsUserAnnotating();
            if (userActivelyAnnotating)
            {
                UpdateAnnotationToolMousePosition(Input.mousePosition, Input.GetMouseButton((int)MouseButton.LeftMouse), Input.GetMouseButtonUp((int)MouseButton.LeftMouse));
            }
            else if (IsUserZooming())
            {
                ComputeZoom();
            }
            else if (IsUserPanning())
            {
                ComputePan();
            }
            else if (IsUserOrbiting())
            {
                ComputeRotate();
            }
            else
            {
                // If we're in reticle mode and the user has actually placed the reticle
                // Look at the reticle
                if (ReticleMode)
                {
                    ComputeReticle();
                }

                lastMousePosition = Vector3.zero;
            }

            if (!userActivelyAnnotating && InAnnotationMode)
            {
                UpdateAnnotationToolMousePosition(Input.mousePosition, false, false);
            }

            // For any zoom / pan / orbit operation, if the reticle has been placed, and 
            // we're in the mode, then look at the focus point
            if (ReticleMode && userMovedReticle)
            {
                Vector3 focusPoint = focusReticle.transform.position;
                targetCamera.transform.LookAt(focusPoint);
            }
        }
    }

    public void OnPointerEnter(PointerEventData data)
    {
        isMouseOver = true;
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        mouseCaptured = !eventData.dragging;
    }

    public void ToggleReticleMode()
    {
        ReticleMode = !ReticleMode;
    }

    private (Material, Mesh) GetActiveMaterialAndMesh()
    {
        (Material mat, SubMeshComponents components) = GetActiveMaterialAndSubmeshComponent();
        return (mat, components.data.mesh);
    }

    private (Material, SubMeshComponents) GetActiveMaterialAndSubmeshComponent()
    {
        ViewerManager manager = ViewerManager.Current();
        bool inHQ = GetStateProperty(n => n.ClientInHQMode).Value;
        if (inHQ)
        {
            return (
                manager.hqHoloportViewer.commonHoloportMaterial,
                manager.hqHoloportViewer.SubMeshComponent
            );
        }
        else
        {
            return (
                manager.holoportDecoder.commonHoloportMaterial,
                manager.holoportDecoder.SubMeshComponents[0]
            );
        }
    }

    /// <summary>
    /// Computes collision for the active decoder
    /// </summary>
    private void ComputeCollision()
    {
        (Material mat, SubMeshComponents components) = GetActiveMaterialAndSubmeshComponent();
        components.ComputeCollision();
    }

    private void UpdateKeyboardFlags()
    {
        if (Utils.IsDeviceIndependentControlDown())
        {
            isControlDown = true;
        }
        else if (Utils.IsDeviceIndependentControlUp())
        {
            isControlDown = false;
        }

        if (Input.GetKeyDown(KeyCode.N) && EventSystem.current.currentSelectedGameObject == null)
        {
            InNavigationOverrideMode = !InNavigationOverrideMode;
        }
    }

    private void UpdateMouseFlags()
    {
        if (Input.GetMouseButtonDown((int)MouseButton.LeftMouse))
        {
            mouseDownPosition = Input.mousePosition;
        } 
        else if (!Input.GetMouseButton((int)MouseButton.LeftMouse))
        {
            mouseCaptured = false;
        }
    }

    private void ComputePan()
    {
        if (IsUserPanning())
        {
            bool isPanStart = lastMousePosition.sqrMagnitude == 0;
            if (isPanStart)
            {
                lastMousePosition = Input.mousePosition; // pixel location on screen;

                Vector3 focusTargetCenter = GetFocusCenter();

                // If all else fails, use the center of mass of the model
                panPoint = focusTargetCenter;

                // Input.mousePosition is in screen coordinates, however "targetCamera"'s view is shown on a portion of the screen.
                // This is just converting the screen coordinates to coordinates relative to the target camera's view on the screen.
                Vector2 local;
                if (RectTransformUtility.ScreenPointToLocalPointInRectangle(validInputRectBound, Input.mousePosition, null, out local))
                {
                    Ray mouseRay = targetCamera.ScreenPointToRay(local - validInputRectBound.rect.position);

                    // Try to find the closest model point if the user clicks on it
                    RaycastHit info;
                    if (Physics.Raycast(mouseRay.origin, mouseRay.direction, out info, 100))
                    {
                        panPoint = info.point;
                    }

                    // Well, lets try to find the closest point from the center of mass to the camera
                    else if (Physics.Raycast(targetCamera.transform.position, (focusTargetCenter - targetCamera.transform.position).normalized, out info, 100))
                    {
                        panPoint = info.point;
                    }
                }
            }

            float focusDistance = Vector3.Project(panPoint - targetCamera.transform.position, targetCamera.transform.forward).magnitude;

            // In summary, this tries to keep the model in line with mouse movements
            // i.e. When you move the mouse, the model should stay the same distance away from the mouse pointer.

            // normalized 0 -> 1 offset
            Vector2 mouseOffset = targetCamera.ScreenToViewportPoint(Input.mousePosition - lastMousePosition);

            // Figures out how big the frustum plane is in world coordinates at **focusDistance** away from the camera
            float frustumHeight = focusDistance * Mathf.Tan(targetCamera.fieldOfView * 0.5f * Mathf.Deg2Rad) * SettingsManager.Instance.CameraPanningSpeed * 2.0f;
            float frustumWidth = frustumHeight * targetCamera.aspect;

            // Translate the mouse coordinates to world coordinates
            Vector2 offset = new Vector2(
                mouseOffset.x * frustumWidth,
                mouseOffset.y * frustumHeight
            );

            // Move the camera
            targetCamera.transform.Translate(-offset, Space.Self);
            lastMousePosition = Input.mousePosition;
        }
    }

    private void OnDrawGizmos()
    {
        if (panPoint.sqrMagnitude > 0)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawCube(panPoint, Vector3.one * 0.05f);
        }
    }

    private void ComputeZoom()
    {
        if (IsUserToolZooming())
        {
            float zoomStrength = (isControlDown ? -1 : 1) * SettingsManager.Instance.ZoomToolScaleFactor;
            ZoomTowards(zoomStrength, Input.mousePosition);
        }
        else if (IsUserDragZooming())
        {
            bool isStartZoom = lastMousePosition.sqrMagnitude == 0.0;
            if (isStartZoom)
            {
                lastMousePosition = Input.mousePosition;
            } 
            else 
            {
                // Scale the amount the mouse has moved according to the window size.
                float mouseDiff = (Input.mousePosition.x - lastMousePosition.x) / validInputRectBound.rect.width;
                float zoomStrength = mouseDiff * SettingsManager.Instance.CameraZoomSpeed;
                targetCamera.transform.Translate(targetCamera.transform.forward * zoomStrength, Space.World);
                lastMousePosition = Input.mousePosition; // pixel location on screen;
            }
        }
        else if (IsUserMouseWheelZooming())
        {
            float zoomStrengh = Input.mouseScrollDelta.y * SettingsManager.Instance.CameraZoomSpeed;
            targetCamera.transform.Translate(targetCamera.transform.forward * zoomStrengh, Space.World);
        }
    }

    private void ComputeRotate()
    {
        // Rotating
        if (IsUserOrbiting())
        {
            bool isStartOrbit = lastMousePosition.sqrMagnitude == 0.0;
            if (isStartOrbit)
            {
                lastMousePosition = Input.mousePosition;
            }
            else
            {
                Vector3 dir = targetCamera.transform.up - focusTarget.up;
                bool isCameraUpsideDown = dir.y < -1.0 && dir.y > -2.0;

                // User dragging across the entire screen should rotate 360 deg
                Vector3 delta = targetCamera.ScreenToViewportPoint(Input.mousePosition - lastMousePosition);
                delta *= SettingsManager.Instance.CameraRotationSpeed * 360.0f;
                Vector3 focusPoint = GetFocusCenter();

                Quaternion deltaRotation =
                    // If the camera is upside down, flip the direction of the left to right
                    Quaternion.AngleAxis((isCameraUpsideDown ? -1 : 1) * delta.x, focusTarget.up) *
                    Quaternion.AngleAxis(-delta.y, targetCamera.transform.right);

                // Compute the new position of the camera around the focus point, based on how much the user rotated.
                targetCamera.transform.position = deltaRotation * (targetCamera.transform.position - focusPoint) + focusPoint;

                // Take the delta rotation and "add" it to the existing rotation of the camera
                targetCamera.transform.rotation = deltaRotation * targetCamera.transform.rotation;

                lastMousePosition = Input.mousePosition;
            }
        }
    }

    private void ComputeReticle()
    {
        if (ReticleMode && 
            Input.GetMouseButtonUp((int)MouseButton.LeftMouse) &&
            (mouseDownPosition - Input.mousePosition).magnitude < 5)
        {
            MoveReticle(Input.mousePosition);
        }
    }

    private bool MoveReticle(Vector3 screenPos)
    {
        bool result = false;
        // If the user barely moved the mouse
        if (ReticleMode && focusReticle != null)
        {
            ComputeCollision();

            Vector3 focusTargetPoint;
            if (RaycastFocusTarget(screenPos, out focusTargetPoint))
            {
                userMovedReticle = true;
                result = true;
                
                // Turn on the reticle as soon as the user selected a point
                focusReticle.gameObject.SetActive(true);

                focusReticle.transform.position = focusTargetPoint;
            }
        }
        UpdateViewTooltip();
        return result;
    }

    private Vector3 GetFocusCenter()
    {
        if (ReticleMode)
        {
            return focusReticle.transform.position;
        }
        else
        {
            return focusTarget.gameObject.ComputeCenter();
        }
    }

    private void ZoomTowards(float zoomStrength, Vector3 screenPos)
    {
        Vector3 pos;
        if (RaycastFocusTarget(screenPos, out pos))
        {
            targetCamera.transform.LookAt(pos);
        }
        targetCamera.transform.Translate(targetCamera.transform.forward * zoomStrength, Space.World);
    }

    /// <summary>
    /// Checks to see if the user is actively rotating
    /// </summary>
    /// <returns></returns>
    private bool IsUserOrbiting()
    {
        return IsUserDragOrbiting() || IsUserToolOrbiting();
    }

    /// <summary>
    /// Returns true if the user is actively orbiting
    /// </summary>
    /// <returns></returns>
    private bool IsUserDragOrbiting()
    {
        return Input.GetMouseButton((int)MouseButton.LeftMouse);
    }

    /// <summary>
    /// Returns true if the user is using the orbit tool to orbit
    /// </summary>
    /// <returns></returns>
    private bool IsUserToolOrbiting()
    {
        return (activeViewerTool == ViewerTool.Orbit && Input.GetMouseButton((int)MouseButton.LeftMouse));
    }

    /// <summary>
    /// Checks to see if the user is actively panning
    /// </summary>
    /// <returns></returns>
    private bool IsUserPanning()
    {
        return IsUserMouseWheelPanning() || IsUserToolPanning() || IsUserDragPanning();
    }

    /// <summary>
    /// Returns true if the user is using the annotation tool
    /// </summary>
    /// <returns></returns>
    private bool IsUserAnnotating()
    {
        return (
            InAnnotationMode && 
            !InNavigationOverrideMode && 
            annotationToolScript.SelectedDrawTool != DrawTool.Default &&
            (Input.GetMouseButton((int)MouseButton.LeftMouse) || Input.GetMouseButtonUp((int)MouseButton.LeftMouse))
        );
    }

    /// <summary>
    /// Returns true if the user is using the pan tool to pan
    /// </summary>
    /// <returns ></returns>
    private bool IsUserDragPanning()
    {
        return (activeViewerTool == ViewerTool.None && Input.GetKey(KeyCode.LeftShift) && Input.GetMouseButton((int)MouseButton.LeftMouse));
    }

    /// <summary>
    /// Returns true if the user is using the pan tool to pan
    /// </summary>
    /// <returns></returns>
    private bool IsUserToolPanning()
    {
        return (activeViewerTool == ViewerTool.Pan && Input.GetMouseButton((int)MouseButton.LeftMouse));
    }

    /// <summary>
    /// Returns true if the user is using the mouse wheel to pan
    /// </summary>
    /// <returns></returns>
    private bool IsUserMouseWheelPanning()
    {
        return (activeViewerTool == ViewerTool.None && Input.GetMouseButton((int)MouseButton.MiddleMouse));
    }

    /// <summary>
    /// Checks to see if the user is actively zooming
    /// </summary>
    /// <returns></returns>
    private bool IsUserZooming()
    {
        return IsUserToolZooming() || IsUserDragZooming() || IsUserMouseWheelZooming();
    }

    /// <summary>
    /// Returns true if the user is zooming though the zoom tool
    /// </summary>
    /// <returns></returns>
    private bool IsUserToolZooming()
    {
        return activeViewerTool == ViewerTool.Zoom && Input.GetMouseButton((int)MouseButton.LeftMouse);
    }

    /// <summary>
    /// Returns true if the user is drag zooming
    /// </summary>
    /// <returns></returns>
    private bool IsUserDragZooming()
    {
        return (Input.GetKey(KeyCode.LeftControl) && Input.GetMouseButton((int)MouseButton.LeftMouse));
    }

    /// <summary>
    /// Returns true if the user is mouse wheel zooming
    /// </summary>
    /// <returns></returns>
    private bool IsUserMouseWheelZooming()
    {
        return Mathf.Abs(Input.mouseScrollDelta.y) > 0;
    }

    private void SizeAndPositionReticle()
    {
        Vector3 reticleScale = Vector3.zero;
        Vector3 position = Vector3.zero;
        if (ReticleMode)
        {
            reticleScale = new Vector3(0.1f, 0.1f, 0.1f);
            if (focusTarget != null)
            {
                Bounds? b = focusTarget.gameObject.ComputeBounds();
                if (b != null)
                {
                    float maxSize = Mathf.Max(b.Value.size.x, b.Value.size.y, b.Value.size.z) / 100;
                    reticleScale = new Vector3(maxSize, maxSize, maxSize);
                    position = b.Value.center;
                } 
                else
                {
                    position = focusTarget.position;
                }
            }
        }
        focusReticle.transform.localScale = reticleScale;
        focusReticle.transform.position = position;
    }

    //overload it to pass hitInfo optionally
    private bool RaycastFocusTarget(Vector3 screenPos, out Vector3 point)
    {
        RaycastHit hitInfo;
        return RaycastFocusTarget(screenPos, out point, out hitInfo);
    }
    
    private bool RaycastFocusTarget(Vector3 screenPos, out Vector3 point, out RaycastHit hitInfo)
    {
        //ScreenPointToLocalPointInRectangle(RectTransform rect, Vector2 screenPoint, Camera cam, out Vector2 localPoint);
        Vector2 localPoint;
        RectTransformUtility.ScreenPointToLocalPointInRectangle((RectTransform)transform, screenPos, null, out localPoint);

        var rect = ((RectTransform)transform).rect;

        // Should compute this
        var pivot = new Vector2(0.5f, 0.5f);

        Ray r = targetCamera.ViewportPointToRay(pivot + (localPoint / rect.size));
        Debug.DrawRay(r.origin, r.direction, Color.white, 5);
        if (Physics.Raycast(r, out hitInfo))
        {
            point = hitInfo.point;
            return true;
        } 
        else
        {
            point = Vector3.zero;
        }

        return false;
    }

    public void ToggleAnnotationMode()
    {
        InAnnotationMode = !InAnnotationMode;
    }

    private bool UpdateAnnotationToolMousePosition(Vector3? screenPos, bool mouseDown, bool mouseUp)
    {
        Vector3 point;
        RaycastHit hitInfo;
        if(screenPos != null && RaycastFocusTarget(screenPos.Value, out point, out hitInfo))
        {
            // we hit something
            annotationToolScript.UpdateMousePosition(transform as RectTransform, screenPos, hitInfo, mouseCaptured, mouseDown, mouseUp);
            return true;
        }
        else
        {
            annotationToolScript.UpdateMousePosition(transform as RectTransform, screenPos, null, mouseCaptured, mouseDown, mouseUp);
            return false;
        }
    }
    private void OnActiveToolChanged(ViewerTool value, ViewerTool oldValue)
    {
        // ViewerManager.Current().Store.GetState().ActiveTool.Value is expensive, cache this
        activeViewerTool = value;
        UpdateViewTooltip();
    }

    /// <summary>
    /// Updates the tooltip displayed on the view
    /// </summary>
    private void UpdateViewTooltip()
    {
        var cursorTooltip = GetComponent<TooltipDisplay>();
        var tool = ViewerManager.Current().Store.GetState().ActiveTool.Value;
        if (cursorTooltip)
        {
            // We only show the focus tooltip if we're in focus mode
            // and the user hasn't already seen the tooltip
            // and they're not currently using a tool
            bool showFocusTooltip = ReticleMode && !userMovedReticle && tool == ViewerTool.None;
            cursorTooltip.Tooltip = showFocusTooltip ?
                FocusTooltip :
                TooltipData.Empty;

        }
    }
}
