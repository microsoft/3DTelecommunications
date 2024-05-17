
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;// Required when using Event data.
using TMPro;

public class TextAnnotationBehavior : MonoBehaviour

{
    /// <summary>
    /// Event for when the text annotation is destroyed
    /// </summary>
    public event UnityAction<string> Destroyed;

    /// <summary>
    /// The color of the text annotation
    /// </summary>
    private Color _selectedColor;

    /// <summary>
    /// The TMPro input field
    /// </summary>
    [SerializeField]
    public GameObject text;

    /// <summary>
    /// The actual TMPro Text Box
    /// </summary>
    [SerializeField]
    public GameObject multiLineInput;
 
    /// <summary>
    /// Gets/sets the color to use when creating the text annotation
    /// </summary>

    public Color SelectedColor
    {
        get
        {
            return _selectedColor;
        }
        set
        {
            
            _selectedColor = value;
            if (text)
            {
                var textInput = text.GetComponentsInChildren<TMPro.TextMeshProUGUI>();
                var m = multiLineInput.GetComponentsInChildren<TMPro.TextMeshProUGUI>();
                if (textInput.Length > 0)
                {
                    UpdateTextColor(textInput[0], value);
                }
                if (m.Length > 0)
                {
                    UpdateTextColor(m[0], value);
                }
            }
            else
            {
                Debug.Log("Missing text gameObject in TextAnnotationBehavior");
            }
        }
    }

    /// <summary>
    /// Updates the text color on the GameObject
    /// </summary>
    /// <param name="inputInstance"></param>
    /// <param name="value"></param>
    private void UpdateTextColor(TextMeshProUGUI inputInstance, Color value)
    {
        inputInstance.faceColor = value;
        inputInstance.color = value;
        inputInstance.fontSharedMaterial.color = value;
    }

    /// <summary>
    /// Focuses the input text box
    /// </summary>
    public void Focus()
    {
        EventSystem.current.SetSelectedGameObject(multiLineInput.gameObject, null);
    }

    /// <summary>
    /// Removes this text annotation
    /// </summary>
    public void Remove()
    {   
        Destroyed?.Invoke(this.name);
        Destroy(this.gameObject);
    }

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    private void Update()
    {
        if (Input.GetKeyUp(KeyCode.Delete) && EventSystem.current.currentSelectedGameObject == multiLineInput)
        {
            Remove();
        }
    }
}
