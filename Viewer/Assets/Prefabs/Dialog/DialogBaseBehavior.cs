using UnityEngine;
using UnityEngine.UI;

[ExecuteInEditMode]
public class DialogBaseBehavior : MonoBehaviour
{
    //[SerializeField]
    //public GameObject footerComponent;

    //[SerializeField]
    //public UnityEvent OnCloseClick;

    [Header("Prefab UI Binding (Dont touch unless editing prefab)")]

    [SerializeField]
    private GameObject contentContainerComponent;

    [SerializeField]
    private GameObject headerContainerComponent;

    [SerializeField]
    private Text headerTextComponent;

    [SerializeField]
    private GameObject footerContainerComponent;

    // Update is called once per frame
    void Update()
    {
        //if (this.footerContainerComponent)
        //{
        //    this.footerContainerComponent.transform.DetachChildren();

        //    if (this.footerComponent != null)
        //    {
        //        this.footerComponent.transform.parent = this.footerContainerComponent.transform;
        //    }
        //}
    }

}
