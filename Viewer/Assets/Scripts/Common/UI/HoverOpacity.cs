using Assets.Scripts.Common.Extensions;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class HoverOpacity : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    [SerializeField]
    private CanvasGroup target;

    [SerializeField]
    private float defaultOpacity = 0.33f;

    [SerializeField]
    private float hoverOpacity;

    [SerializeField]
    private float animationDurationMS = 200;

    private bool isPointerOver = false;

    /// <summary>
    /// The time that the pointer entered the panel
    /// </summary>
    private DateTime pointerEnterTime = DateTime.MinValue;

    /// <summary>
    /// The time that the pointer exited the panel
    /// </summary>
    private DateTime pointerExitTime = DateTime.MinValue;

    // Start is called before the first frame update
    void Start()
    {
        if (target == null)
        {
            target = GetComponent<CanvasGroup>();
        }
    }

    /// <summary>
    /// Lifecyle method for when this behavior is enabled
    /// </summary>
    void OnEnable()
    {
        if (target != null)
        {
            // Setup the default opacity
            target.alpha = defaultOpacity;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (target != null)
        {
            double pointerDelta;
            if (isPointerOver)
            {
                pointerDelta = (DateTime.Now - pointerEnterTime).TotalMilliseconds;
            }
            else
            {
                pointerDelta = (DateTime.Now - pointerExitTime).TotalMilliseconds;
            }

            if (pointerDelta < animationDurationMS)
            {
                float targetOpacity = isPointerOver ? hoverOpacity : defaultOpacity;
                target.alpha = target.alpha + ((targetOpacity - target.alpha) * ((float)pointerDelta / animationDurationMS));
            }
        }
    }

    /// <summary>
    /// Handler for when the mouse enters this component
    /// </summary>
    /// <param name="eventData"></param>
    public void OnPointerEnter(PointerEventData eventData)
    {
        pointerEnterTime = DateTime.Now;
        isPointerOver = true;
    }

    /// <summary>
    /// Handler for when the mouse exits this component
    /// </summary>
    /// <param name="eventData"></param>
    public void OnPointerExit(PointerEventData eventData)
    {
        #if UNITY_2021_2_OR_NEWER // In unity 2021.2ish, they switched the default behavior
        // fullyExited == if the pointer is completely off of this element. OnPointerExit still gets called if you hover off of this element, but onto a child element.
        if (!eventData.fullyExited)
        {
            return;
        }
        #endif

        pointerExitTime = DateTime.Now;
        isPointerOver = false;
    }
}
