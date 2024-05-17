using System;
using UnityEngine;
using UnityEngine.UI;

[ExecuteInEditMode]
public class IconButtonBehavior : MonoBehaviour
{
    [SerializeField]
    public string text;

    [SerializeField]
    public Sprite icon;

    [SerializeField]
    public float iconScale = 1.0f;

    [SerializeField]
    public string iconText;

    [Header("Prefab UI Binding")]
    [SerializeField]
    private RectTransform iconContainer;

    [SerializeField]
    private Image iconComponent;

    [SerializeField]
    private Text iconTextComponent;

    [SerializeField]
    private Text textComponent;


    void Update()
    {
        if (this.iconContainer != null)
        {
            this.iconScale = Mathf.Clamp(this.iconScale, 0, 1);
            this.iconContainer.localScale = new Vector3(iconScale, iconScale, iconScale);
        }

        if (this.iconComponent != null)
        {
            this.iconComponent.sprite = this.icon;
        }

        if (this.iconTextComponent)
        {
            this.iconTextComponent.text = this.iconText;
        }

        if (this.textComponent != null)
        {
            this.textComponent.text = this.text;
        }

        // We're using a text icon
        if (this.icon != null)
        {
            this.SetActive(this.iconComponent, true);
            this.SetActive(this.iconTextComponent, false);
        }
        else if (this.iconText != null)
        {
            this.SetActive(this.iconComponent, false);
            this.SetActive(this.iconTextComponent, true);
        }
    }

    private void SetActive(Component c, bool active)
    {
        if (c != null)
        {
            c.gameObject.SetActive(active);
        }
    }
}
