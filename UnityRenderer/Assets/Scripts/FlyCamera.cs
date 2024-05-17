using System;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Rendering;
using UnityEngine.UIElements;

public class FlyCamera : MonoBehaviour
{
    // what we use as rotational pivot
    public Camera targetCamera;
    public GameObject focusReticle;

    public float CameraPanningSpeed = 0.1f;
    public float CameraZoomSpeed = 0.1f;
    public float CameraRotationSpeed = 0.5f;
    public Vector3 focusCenter = Vector3.zero;

    // area in which we can control the camera
    public RectTransform validInputRectBound;

    private float camFOV;
    private Vector3 mouseDownPosition;
    private Vector3 lastMousePosition;
    private Vector3 camStartPosition;
    private Vector3 panPoint;
    private Quaternion camStartRotation;
    private bool userMovedReticle = false;

    bool isControlDown = false;

    // Start is called before the first frame update
    void OnEnable()
    {
        lastMousePosition = Vector3.zero;

        camStartPosition = targetCamera.transform.position;
        camStartRotation = targetCamera.transform.rotation;
        camFOV = targetCamera.fieldOfView;
    }

    public void ResetCamera()
    {
        targetCamera.transform.position = camStartPosition;
        targetCamera.transform.rotation = camStartRotation;
        targetCamera.fieldOfView = camFOV;
    }

    void Update()
    {
        //check for valid area input
        Vector2 mouseInRectPos = validInputRectBound.InverseTransformPoint(Input.mousePosition);
        //dont update if that bound is not active in UI hierarchy OR mouse is not withint bounds
        if (!validInputRectBound.gameObject.activeInHierarchy || !validInputRectBound.rect.Contains(mouseInRectPos))
        {
            return;
        }

        UpdateKeyboardFlags();
        UpdateMouseFlags();

        if (IsUserZooming())
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
            lastMousePosition = Vector3.zero;
        }
    }

    private void UpdateKeyboardFlags()
    {
        if (Input.GetKeyDown(KeyCode.LeftControl))
        {
            isControlDown = true;
        }
        else if (Input.GetKeyUp(KeyCode.LeftControl))
        {
            isControlDown = false;
        }
    }

    private void UpdateMouseFlags()
    {
        if (Input.GetMouseButtonDown((int)MouseButton.LeftMouse))
        {
            mouseDownPosition = Input.mousePosition;
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

                Vector3 focusTargetCenter = focusCenter;

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
            float frustumHeight = focusDistance * Mathf.Tan(targetCamera.fieldOfView * 0.5f * Mathf.Deg2Rad) * CameraPanningSpeed * 2.0f;
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
        if (IsUserDragZooming())
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
                float zoomStrength = mouseDiff * CameraZoomSpeed;
                targetCamera.transform.Translate(targetCamera.transform.forward * zoomStrength, Space.World);
                lastMousePosition = Input.mousePosition; // pixel location on screen;
            }
        }
        else if (IsUserMouseWheelZooming())
        {
            float zoomStrengh = Input.mouseScrollDelta.y * CameraZoomSpeed;
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
                Vector3 dir = targetCamera.transform.up - Vector3.up;
                bool isCameraUpsideDown = dir.y < -1.0 && dir.y > -2.0;

                // User dragging across the entire screen should rotate 360 deg
                Vector3 delta = targetCamera.ScreenToViewportPoint(Input.mousePosition - lastMousePosition);
                delta *= CameraRotationSpeed * 360.0f;
                Vector3 focusPoint = focusCenter;

                Quaternion deltaRotation =
                    // If the camera is upside down, flip the direction of the left to right
                    Quaternion.AngleAxis((isCameraUpsideDown ? -1 : 1) * delta.x, Vector3.up) *
                    Quaternion.AngleAxis(-delta.y, targetCamera.transform.right);

                // Compute the new position of the camera around the focus point, based on how much the user rotated.
                targetCamera.transform.position = deltaRotation * (targetCamera.transform.position - focusPoint) + focusPoint;

                // Take the delta rotation and "add" it to the existing rotation of the camera
                targetCamera.transform.rotation = deltaRotation * targetCamera.transform.rotation;

                lastMousePosition = Input.mousePosition;
            }
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
        return false;
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
    /// Returns true if the user is using the pan tool to pan
    /// </summary>
    /// <returns ></returns>
    private bool IsUserDragPanning()
    {
        return false;
    }

    /// <summary>
    /// Returns true if the user is using the pan tool to pan
    /// </summary>
    /// <returns></returns>
    private bool IsUserToolPanning()
    {
        return false;
    }

    /// <summary>
    /// Returns true if the user is using the mouse wheel to pan
    /// </summary>
    /// <returns></returns>
    private bool IsUserMouseWheelPanning()
    {
        return Input.GetMouseButton((int)MouseButton.MiddleMouse);
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
        return false;
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
}
