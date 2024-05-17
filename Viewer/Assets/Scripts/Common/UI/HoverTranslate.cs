using Assets.Scripts.Common;
using Assets.Scripts.Common.Extensions;
using System;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using System.Collections;
using Assets.Scripts.Events;

public enum MenuDirection
{
    Left,
    Right
}
public class HoverTranslate : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    [SerializeField]
    public float collapsedWidth = 15f;

    [SerializeField]
    public float expandedWidth = 100f;

    [SerializeField]
    public float collapsedOpacity = 1f;

    [SerializeField]
    public float expandedOpacity = 1f;

    [SerializeField]
    public long animationTimeMS = 200;

    private float _posOffset = 0;


    [SerializeField]

    public MenuDirection menuPosition = MenuDirection.Right;

    [SerializeField]
    public BoolEvent OnExpandedChanged;

    [SerializeField]
    public long expandDelayMS = 1000;

    private Easing<float> collapseEaseFn;
    private Easing<float> expandEaseFn;

    private bool _isHovered = false;
    private bool easeComplete = true;
    private DateTime easeBegin;


    private float StartingXPosition = 0;

    private bool IsHovered
    {
        get
        {
            return this._isHovered;
        }
        set
        {
            CanvasGroup group = this.gameObject.GetComponent<CanvasGroup>();
            this._isHovered = value;
            if (group != null)
            {
                group.alpha = value ? this.expandedOpacity : this.collapsedOpacity;
            }
        }
    }

    void Start()
    {
        var rect = transform.GetComponent<RectTransform>();
        if (rect != null)
        {
            this.StartingXPosition = rect.transform.position.x;
        }


        this._posOffset = this.expandedWidth - this.collapsedWidth - 15;
    }


    IEnumerator ExpandRoutine()
    {
        float duration = this.animationTimeMS / 1000;
        float stratingx = this.StartingXPosition;
        float endX = this.StartingXPosition - this._posOffset;
        float t = 0.0f;
        while (t <= duration)
        {
            t += Time.deltaTime;
            float xPos = Mathf.Lerp(stratingx, endX, t / duration);
            transform.position = new Vector3(xPos, transform.position.y, transform.position.z);
            yield return null;
        }
        var rect = transform.GetComponent<RectTransform>();
        if (rect != null)
        {
            rect.position = new Vector3(endX, transform.position.y, transform.position.z);

        }

    }

    IEnumerator Contractoutine()
    {
        float duration = this.animationTimeMS / 1000;

        float stratingx = transform.position.x;
        float endX = this.StartingXPosition;
        float t = 0.0f;
        while (t <= duration)
        {
            t += Time.deltaTime;
            float xPos = Mathf.Lerp(stratingx, endX, t / duration);
            transform.position = new Vector3(xPos, transform.position.y, transform.position.z);
            yield return null;
        }
        var rect = transform.GetComponent<RectTransform>();
        if (rect != null)
        {
            // this.StartingXPosition = rect.transform.position.x;
            rect.position = new Vector3(endX, transform.position.y, transform.position.z);

        }

    }


    public void OnPointerEnter(PointerEventData eventData)
    {
        IsHovered = true;
        easeComplete = false;
        StartCoroutine(ExpandRoutine());
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        IsHovered = false;
        easeComplete = false;
        StartCoroutine(Contractoutine());
    }
}
