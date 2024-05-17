using Assets.Scripts.Common;
using Assets.Scripts.Common.Extensions;
using Assets.Scripts.Events;
using System;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;

public class HoverExpand : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler
{
    [SerializeField]
    public float collapsedWidth = 15f;

    [SerializeField]
    public float expandedWidth = 15f;

    [SerializeField]
    public float collapsedOpacity = 1f;

    [SerializeField]
    public float expandedOpacity = 1f;

    [SerializeField]
    public long animationTimeMS = 200;

    [SerializeField]
    public float posOffset = 0;

    [SerializeField]
    public EaseType expandEase = EaseType.CubicOut;

    [SerializeField]
    public EaseType collapseEase = EaseType.CubicIn;

    [SerializeField]
    public RectTransform.Axis direction = RectTransform.Axis.Horizontal;

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
        // Start Collapsed
        this.SetSize(collapsedWidth);
        if (this.OnExpandedChanged != null)
        {
            this.OnExpandedChanged.Invoke(false);
        }
        this.StartingXPosition = transform.position.x;
    }

    void Update()
    {
        var endExpansionX = this.StartingXPosition;
        if (!easeComplete)
        {
            EasingResult<float> result;
            if (IsHovered)
            {
                endExpansionX = this.StartingXPosition - this.posOffset;
                collapseEaseFn = null;
                if (expandEaseFn == null)
                {
                    var progress = Mathf.Clamp((this.GetSize() - collapsedWidth) / (expandedWidth - collapsedWidth), 0, 1);
                    expandEaseFn = collapsedWidth.Ease(expandedWidth, DateTime.Now, animationTimeMS - (long)(animationTimeMS * progress), expandEase, expandDelayMS);
                }

                result = expandEaseFn(DateTime.Now);
            }
            else
            {
                expandEaseFn = null;
                if (collapseEaseFn == null)
                {
                    var progress = 1.0 - Mathf.Clamp((this.GetSize() - collapsedWidth) / (expandedWidth - collapsedWidth), 0, 1);
                    collapseEaseFn = expandedWidth.Ease(collapsedWidth, DateTime.Now, animationTimeMS - (long)(animationTimeMS * progress), collapseEase);
                }

                result = collapseEaseFn(DateTime.Now);
            }

            if (!result.complete)
            {
                SetSize(result.value);
            }
            else
            {
                SetSize(result.value);
                easeComplete = true;
                float endXPos = this.posOffset;

                transform.position = new Vector3(endExpansionX, transform.transform.position.y, transform.position.z);
                if (OnExpandedChanged != null)
                {
                    OnExpandedChanged.Invoke(this.IsHovered);
                }
            }
        }
    }

    private float GetSize()
    {
        if (gameObject.transform is RectTransform)
        {
            var transform = (RectTransform)gameObject.transform;
            if (direction == RectTransform.Axis.Horizontal)
            {
                return transform.rect.width;
            }
            else
            {
                return transform.rect.height;
            }
        }
        return 0;
    }

    private void SetSize(float size)
    {
        if (gameObject.transform is RectTransform)
        {
            ((RectTransform)gameObject.transform).SetSizeWithCurrentAnchors(direction, size);
        }
    }

    public void OnPointerEnter(PointerEventData eventData)
    {
        IsHovered = true;
        easeComplete = false;
    }

    public void OnPointerExit(PointerEventData eventData)
    {
        IsHovered = false;
        easeComplete = false;
    }
}
