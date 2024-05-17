using UnityEngine;
using System;
using UnityEngine.UIElements;
using UnityEngine.EventSystems;
using Assets.Scripts.Common;

public class FlyCamera : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler, IPointerDownHandler, IPointerUpHandler
{

    /*
    Writen by Windexglow 11-13-10.  Use it, edit it, steal it I don't care.  
    Converted to C# 27-02-13 - no credit wanted.
    Simple flycam I made, since I couldn't find any others made public.  
    Made simple to use (drag and drop, done) for regular keyboard layout  
    wasd : basic movement
    shift : Makes camera accelerate
    space : Moves camera on X and Z axis only.  So camera doesn't gain any height*/

    [SerializeField]
    public float mainSpeed = 10.0f; //regular speed

    [SerializeField]
    public bool cameraLook = false;

    [SerializeField]
    public float shiftMultiplier = 2.5f; //multiplied by how long shift is held.  Basically running

    [SerializeField]
    public float maxShift = 250.0f; //Maximum speed when holdin gshift

    [SerializeField]
    Renderer boundsObj;

    public ITransformable transformable;

    float camSens = 0.25f; //How sensitive it with mouse
    private Vector3 lastMouse = new Vector3(255, 255, 255); //kind of in the middle of the screen, rather than at the top (play)
    private float totalRun = 1.0f;

    bool isMouseDown = false;
    bool isMouseOver = false;

    void Update()
    {
        if ((isMouseOver || isMouseDown) && transformable != null)
        {
            var transform = transformable;
            if (cameraLook && isMouseDown)
            {
                lastMouse = Input.mousePosition - lastMouse;
                lastMouse = new Vector3(-lastMouse.y * camSens, lastMouse.x * camSens, 0);
                lastMouse = new Vector3(transform.Rotation.eulerAngles.x + lastMouse.x, transformable.Rotation.eulerAngles.y + lastMouse.y, 0);
                transformable.Rotation = Quaternion.Euler(lastMouse);

                lastMouse = Input.mousePosition;
                //Mouse  camera angle done.  
            }

            Bounds bounds = new Bounds();
            if (this.boundsObj != null) {
                bounds = this.boundsObj.bounds;
            }

            //Keyboard commands
            float f = 0.0f;
            Vector3 p = GetBaseInput();
            if (Input.GetKey(KeyCode.LeftShift))
            {
                totalRun += Time.deltaTime;
                p = p * totalRun * shiftMultiplier;
                p.x = Mathf.Clamp(p.x, -maxShift, maxShift);
                p.y = Mathf.Clamp(p.y, -maxShift, maxShift);
                p.z = Mathf.Clamp(p.z, -maxShift, maxShift);
            }
            else
            {
                totalRun = Mathf.Clamp(totalRun * 0.5f, 1f, 1000f);
                p = p * mainSpeed;
            }

            p = p * Time.deltaTime;
            Vector3 newPosition = transform.Position;
            if (Input.GetKey(KeyCode.Space))
            { //If player wants to move on X and Z axis only
                transform.Position = transform.Position + (transform.Rotation * p);
                newPosition.x = transform.Position.x;
                newPosition.z = transform.Position.z;
                transform.Position = newPosition;
            }
            else
            {
                transform.Position = transform.Position + (transform.Rotation * p);
            }

            if (bounds.size.x > 0 && bounds.size.y > 0 && bounds.size.z > 0) {
                Vector3 position = transform.Position;
                position.x = Mathf.Clamp(position.x, bounds.min.x, bounds.max.x);
                position.y = Mathf.Clamp(position.y, bounds.min.y, bounds.max.y);
                position.z = Mathf.Clamp(position.z, bounds.min.z, bounds.max.z);
                transform.Position = position;
            }
        }
    }

    private Vector3 GetBaseInput()
    { //returns the basic values, if it's 0 than it's not active.
        Vector3 p_Velocity = new Vector3();
        if (Input.GetKey(KeyCode.W))
        {
            p_Velocity += new Vector3(0, 0, 1);
        }
        if (Input.GetKey(KeyCode.S))
        {
            p_Velocity += new Vector3(0, 0, -1);
        }
        if (Input.GetKey(KeyCode.A))
        {
            p_Velocity += new Vector3(-1, 0, 0);
        }
        if (Input.GetKey(KeyCode.D))
        {
            p_Velocity += new Vector3(1, 0, 0);
        }
        return p_Velocity;
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        if (eventData.button != PointerEventData.InputButton.Left)
        {
            this.isMouseOver = true;
        }
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        this.isMouseOver = false;
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        if (eventData.button == PointerEventData.InputButton.Left) {  
           isMouseDown = true;
           this.lastMouse = Input.mousePosition;
        }
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        if (eventData.button == PointerEventData.InputButton.Left)
        {
            isMouseDown = false;
            this.lastMouse = Input.mousePosition;
        }
    }
}