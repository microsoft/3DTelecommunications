using Assets.Scripts.Common;
using UnityEngine;
using UnityEngine.UI;

public class SpotlightBehavior : Graphic
{
    [Range(0.01f, 1.0f)]
    public float spotlightSize = 0.1f;

    [Range(0.0f, 1.0f)]
    public float shadedOpacity = 0.1f;

    private SettingName setting;

    protected override void Start()
    {
        base.Start();
        this.material = new Material(Shader.Find("Custom/Spotlight"));
    }

    protected override void OnPopulateMesh(VertexHelper vh)
    {
        Vector2 corner1 = Vector2.zero;
        Vector2 corner2 = Vector2.zero;

        corner1.x = 0f;
        corner1.y = 0f;
        corner2.x = 1f;
        corner2.y = 1f;

        corner1.x -= rectTransform.pivot.x;
        corner1.y -= rectTransform.pivot.y;
        corner2.x -= rectTransform.pivot.x;
        corner2.y -= rectTransform.pivot.y;

        corner1.x *= rectTransform.rect.width;
        corner1.y *= rectTransform.rect.height;
        corner2.x *= rectTransform.rect.width;
        corner2.y *= rectTransform.rect.height;

        vh.Clear();

        vh.AddVert(new Vector2(corner1.x, corner1.y), color, new Vector2(0, 0));
        vh.AddVert(new Vector2(corner1.x, corner2.y), color, new Vector2(0, 1));
        vh.AddVert(new Vector2(corner2.x, corner2.y), color, new Vector2(1, 1));
        vh.AddVert(new Vector2(corner2.x, corner1.y), color, new Vector2(1, 0));

        vh.AddTriangle(0, 1, 2);
        vh.AddTriangle(2, 3, 0);
    }
    protected override void OnEnable()
    {
        base.OnEnable();
        SettingsManager.Instance.OnChange += OnConfigSettingChange;
        Color maskColor = SettingsManager.Instance.SpotlightMaskColor;
        color = new Color(maskColor.r, maskColor.g, maskColor.b);
        shadedOpacity = maskColor.a;
        this.setting = SettingsManager.Instance.SpotlightMaskColorKey;
    }

    protected override void OnDisable()
    {
        base.OnDisable();
        SettingsManager.Instance.OnChange -= OnConfigSettingChange;
    }

    private void OnConfigSettingChange(SettingName updatedSetting, object value)
    {
        // update current view if settings background changes
        if (this.setting == updatedSetting)
        {
            Vector4 c = (Vector4)value;
            color = new Color(c[0], c[1], c[2]);
            shadedOpacity = c[3];
            Update();
        }
    }

    public void Update()
    {
        Vector2 local;
        if (RectTransformUtility.ScreenPointToLocalPointInRectangle(rectTransform, Input.mousePosition, null, out local))
        {
            this.material.SetVector("_Dimensions", new Vector4(rectTransform.rect.width, rectTransform.rect.height, 0, 0));
            this.material.SetVector("_Mouse", new Vector4(local.x + (rectTransform.pivot.x * rectTransform.rect.width), local.y + (rectTransform.pivot.y * rectTransform.rect.height), 0, 0));
        }

        spotlightSize = SettingsManager.Instance.SpotlightSize;
        this.material.SetColor("_Color", this.color);
        this.material.SetFloat("_ShadedOpacity", this.shadedOpacity);
        this.material.SetFloat("_Size", this.spotlightSize);
    }
}
