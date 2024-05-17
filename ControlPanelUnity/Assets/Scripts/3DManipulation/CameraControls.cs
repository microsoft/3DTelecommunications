using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.UIElements;

public class CameraControls : MonoBehaviour
{
    // what we use as rotational pivot
    public Transform focusTarget;
    public Camera targetCamera;
    // area in which we can control the camera
    public RectTransform validInputRectBound;

    [Header("Control Parameters")]
    public float camPanningSpeed = 3.0f;
    public float camRotationSpeed = 5.0f;
    public float camZoomSpeed = 5.0f;


    private Vector3 mouseStartPosition;
    private Vector3 camStartPosition;
    private Quaternion camStartRotation;
    // Start is called before the first frame update
    void Start()
    {
        mouseStartPosition = Vector3.zero;

        camStartPosition = targetCamera.transform.position;
        camStartRotation = targetCamera.transform.rotation;

        camPanningSpeed = SettingsManager.Instance.GetValueWithDefault("UI", "CameraPanningSpeed", camPanningSpeed);
        camRotationSpeed = SettingsManager.Instance.GetValueWithDefault("UI", "CameraRotationSpeed", camRotationSpeed);
        camZoomSpeed = SettingsManager.Instance.GetValueWithDefault("UI", "CameraZoomSpeed", camZoomSpeed);
    }

    public void ResetCamera()
    {
        targetCamera.transform.position = camStartPosition;
        targetCamera.transform.rotation = camStartRotation;
    }

    // Update is called once per frame
    void Update()
    {
        MouseUpdate();
    }

    void MouseUpdate()
    {

        //check for valid area input
        Vector2 mouseInRectPos = validInputRectBound.InverseTransformPoint(Input.mousePosition);
        //dont update if that bound is not active in UI hierarchy OR mouse is not withint bounds
        if (!validInputRectBound.gameObject.activeInHierarchy || !validInputRectBound.rect.Contains(mouseInRectPos))
        {
            return;
        }


        ///PANNING 
        //middle mouse button first click
        if(Input.GetMouseButtonDown((int)MouseButton.MiddleMouse))
        {
            mouseStartPosition = Input.mousePosition; // pixel location on screen;
        }

        //middle mouse button is heldDown
        bool isPanning = Input.GetMouseButton((int)MouseButton.MiddleMouse);
        if (isPanning)
        {
            //normalized 0~1 offset
            Vector3 mouseOffset = targetCamera.ScreenToViewportPoint(Input.mousePosition - mouseStartPosition);
            mouseOffset *= camPanningSpeed * Time.deltaTime;
            mouseOffset.z = 0; // dont move forward/back
            
            //invert it
            targetCamera.transform.Translate(-mouseOffset, Space.Self);
            mouseStartPosition = Input.mousePosition;
            
        }

        ///ROTATING
        //right mosue button first click
        if (Input.GetMouseButtonDown((int)MouseButton.RightMouse))
        {
            mouseStartPosition = Input.mousePosition; // pixel location on screen;
        }

        bool isRotating = Input.GetMouseButton((int)MouseButton.RightMouse);
        if(isRotating)
        {
            //normalized 0~1 offset
            Vector3 mouseOffset = targetCamera.ScreenToViewportPoint(Input.mousePosition - mouseStartPosition);
            mouseOffset *= camRotationSpeed * Time.deltaTime;
            mouseOffset.z = 0; // dont move forward/back

            targetCamera.transform.RotateAround(focusTarget.position, focusTarget.up, mouseOffset.x );

            targetCamera.transform.RotateAround(focusTarget.position, targetCamera.transform.right, -mouseOffset.y);

            mouseStartPosition = Input.mousePosition;            
        }

        ///ZOOMING

        bool isZooming = Mathf.Abs(Input.mouseScrollDelta.y) > 0;
        if(isZooming)
        {
            float zoomStrengh = Input.mouseScrollDelta.y * Time.deltaTime * camZoomSpeed;
            targetCamera.transform.Translate(targetCamera.transform.forward * zoomStrengh, Space.World);
        }

        Debug.DrawLine(targetCamera.transform.position, targetCamera.transform.position + targetCamera.transform.forward * 5);
    }
}
