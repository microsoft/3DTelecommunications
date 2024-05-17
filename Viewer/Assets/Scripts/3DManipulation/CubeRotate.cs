using Assets.Scripts.Viewer.Common;
using System;
using System.Linq;
using UnityEngine;
using UnityEngine.UIElements;
using Assets.Scripts.Common.Extensions;
using UnityEngine.EventSystems;

public class CubeRotate : MonoBehaviour, IPointerClickHandler
{
    // Start is called before the first frame update
    [SerializeField]
    public Transform focusTarget;

    [SerializeField]
    public Transform sceneTarget;


    [SerializeField]
    public Camera navCamera;

    [SerializeField]
    public Camera sceneCamera;

    [SerializeField]
    private Camera topNavCamera;

    [SerializeField]
    private Camera frontNavCamera;

    [SerializeField]
    private Camera[] cameraCycle;

    private Vector3 mouseStartPosition = Vector3.zero;

    private Vector3 lastScenePosition;
    private Quaternion lastSceneRotation;

    public float camRotationSpeed = 100000.0f;

    private void Update()
    {
        if (lastScenePosition != sceneCamera.transform.position ||
            lastSceneRotation != sceneCamera.transform.rotation)
        {
            lastScenePosition = sceneCamera.transform.position;
            lastSceneRotation = sceneCamera.transform.rotation;

            var sceneTargetCenter = sceneTarget.position;
            var bounds = sceneTarget.gameObject.ComputeBounds();
            if (bounds != null)
            {
                sceneTargetCenter = bounds.Value.center;
            }

            // Get the scene camera position offset relative to the scene target
            var sceneCameraOffset = (sceneCamera.transform.position - sceneTargetCenter);

            // The raw distance away from the center of the scene target
            var sceneCameraDist = sceneCameraOffset.magnitude;

            // Scale the vector components by the total distance, to make all the vector components 0 to 1.
            var scalar = sceneCameraOffset / sceneCameraDist;

            // Figure out how far the nav camera is away from it's focus target
            var navCameraDist = (focusTarget.position - navCamera.transform.position).magnitude;

            // Find the new position by starting at the cubes focus target, and then scaling the x,y,z by the scene camera offset
            var navCameraPos = focusTarget.position + (scalar * navCameraDist);

            // In the end the nav camera is positioned in the relative same spot around it's cube that the scene camera is around its target
            navCamera.transform.position = navCameraPos;

            // Now re-look at the cube
            navCamera.transform.LookAt(focusTarget);
        }
    }

    public void CycleNext()
    {
        Camera nearestCam = FindNearestCamera(sceneCamera);
        int idx = Array.IndexOf(cameraCycle, nearestCam);
        GoToCamera(idx + 1);
    }
    public void CyclePrev()
    {
        Camera nearestCam = FindNearestCamera(sceneCamera);
        int idx = Array.IndexOf(cameraCycle, nearestCam);
        GoToCamera(idx - 1);
    }

    public void MouseUpdate()
    {
        Debug.Log("Dragging...");

        bool isRotating = Input.GetMouseButton((int)MouseButton.RightMouse);
        if (isRotating)
        {

            //normalized 0~1 offset
            Vector3 mouseOffset = navCamera.ScreenToViewportPoint(Input.mousePosition - mouseStartPosition);

            mouseOffset *= camRotationSpeed * Time.deltaTime;
            mouseOffset.z = 0; // dont move forward/back
                               // rotate along the x/y Axis

            // The updates to the cube cam get handled in Update
            sceneCamera.transform.RotateAround(sceneTarget.position, sceneTarget.up, mouseOffset.x);


            mouseStartPosition = Input.mousePosition;

        }
    }

    private void GoToCamera(int idx)
    {
        Camera c = cameraCycle[Utils.Modulo(idx, cameraCycle.Length)];

        // The updates to the cube will get picked up in Update
        sceneCamera.transform.position = c.transform.position;
        sceneCamera.FitIntoView(sceneTarget.gameObject);
    }

    private Camera FindNearestCamera(Camera sourceCam)
    {
        return cameraCycle.OrderBy(n => (n.transform.position - sourceCam.transform.position).magnitude).First();
    }

    public void OnPointerClick(PointerEventData eventData)
    {
        if (eventData.button == PointerEventData.InputButton.Left)
        {
            Camera nearestCam = FindNearestCamera(sceneCamera);
            GoToCamera(Array.IndexOf(cameraCycle, nearestCam));
        }
    }
}


