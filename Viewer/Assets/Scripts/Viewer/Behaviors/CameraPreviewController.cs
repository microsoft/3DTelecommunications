using System;
using UnityEngine;
using UnityEngine.UI;
using Assets.Scripts.Common.Extensions;
using Assets.Scripts.Common;
using HoloportationDecoders.Color2D;
using UnityEngine.Events;

public class CameraPreviewController : MonoBehaviour
{
    private CameraFeedViewer feedViewer;
    private AspectRatioFitter aspectRatioFitter;
    private ICameraFeed _feed;
    private Image selectionBorder;

    /// <summary>
    /// Fires when the preview is clicked
    /// </summary>
    public UnityEvent OnClick;

    /// <summary>
    /// Gets the feed that this preview controller is showing
    /// </summary>
    public ICameraFeed Feed
    {
        get
        {
            return _feed;
        }
        set
        {
            _feed = value;

            UpdateFeed();
        }
    }

    /// <summary>
    /// Set the selection state of the preview
    /// </summary>
    private bool _selected;
    public bool Selected
    {
        get { return _selected; }
        set
        {
            _selected = value;

            UpdateSelectedState();
        }
    }

    public void Start()
    {
        //container.rect.Set(0, 0, previewWidth, previewHeight);

        aspectRatioFitter = GetComponentInChildren<AspectRatioFitter>();
        feedViewer = GetComponentInChildren<CameraFeedViewer>();
        selectionBorder = gameObject.transform.Find("SelectionBorder").GetComponent<Image>();

        // Images are flipped, so re-flip 'em
        ((RectTransform)aspectRatioFitter.transform).localScale = new Vector3(1f, -1f, 1f);

        aspectRatioFitter.aspectMode = AspectRatioFitter.AspectMode.FitInParent;

        var cameraText = GetComponentInChildren<Text>();
        cameraText.text = "";
        cameraText.color = Colorizer.Color(ColorType.Selected);
        cameraText.alignment = TextAnchor.UpperCenter;
        
        //cameraText.font = previewTextFont;
        //cameraText.fontSize = previewTextFontSize;

        //cameraText.resizeTextForBestFit = true;
        cameraText.gameObject.StretchToParent();
        cameraText.gameObject.transform.localScale = Vector3.one;

        cameraText.rectTransform.offsetMax = new Vector2(cameraText.rectTransform.offsetMax.x, -5f);

        var button = GetComponent<Button>();

        button.onClick.AddListener(HandleButtonClick);

        UpdateFeed();
        UpdateSelectedState();
    }

    /// <summary>
    /// Handler for when the preview was clicked
    /// </summary>
    private void HandleButtonClick()
    {
        OnClick?.Invoke();
    }

    /// <summary>
    /// Handler for when the Feed property is updated
    /// </summary>
    private void UpdateFeed()
    {
        if (aspectRatioFitter != null && Feed != null)
        {
            aspectRatioFitter.aspectRatio = (float)Feed.Image.width / Feed.Image.height;
        }

        if (feedViewer != null)
        {
            feedViewer.Feed = Feed;
        }
    }

    /// <summary>
    /// Handler for when the Selected property changes
    /// </summary>
    private void UpdateSelectedState()
    {
        if (selectionBorder != null)
        {
            selectionBorder.enabled = Selected;
        }
    }
}
