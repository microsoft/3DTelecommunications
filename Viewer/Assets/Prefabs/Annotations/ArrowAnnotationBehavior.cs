using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class ArrowAnnotationBehavior : MonoBehaviour
{

    /// <summary>
    /// A reference to the tail section
    /// </summary>
    [SerializeField]
    private RawImage tail;

    /// <summary>
    /// A reference to the head section
    /// </summary>
    [SerializeField]
    private TextMeshProUGUI head;

    /// <summary>
    /// A reference to the close button
    /// </summary>
    [SerializeField]
    private Button closeButton;

    /// <summary>
    /// Event for when the arrrow annotation behavior has been Destroyed
    /// </summary>

    public event UnityAction Destroyed;

    /// <summary>
    /// Gets/sets the arrow color
    /// </summary>
    public Color ArrowColor
    {
        get
        {
            if (tail != null)
            {
                return tail.color;
            }
            return Color.white;
        }
        set
        {
            if (tail != null)
            {
                tail.color = value;
            }
            if (head != null)
            {
                head.color = value;
            }
        }
    }

    /// <summary>
    /// Removes this annotation
    /// </summary>
    public void Remove()
    {
        Destroyed?.Invoke();
        Destroy(gameObject);
    }

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    private void OnEnable()
    {
        closeButton?.onClick.AddListener(OnCloseButtonClick);
    }

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    private void OnDisable()
    {
        closeButton?.onClick.RemoveListener(OnCloseButtonClick);
    }

    /// <summary>
    /// Listener for when the close button is clicked on
    /// </summary>
    private void OnCloseButtonClick()
    {
        Remove();
    }

    /// <summary>
    /// <inheritdoc />
    /// </summary>
    private void Update()
    {
        if (EventSystem.current.currentSelectedGameObject == gameObject && Input.GetKeyDown(KeyCode.Delete))
        {
            Remove();
        }
    }
}
