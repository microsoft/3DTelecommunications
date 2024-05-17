using Assets.Scripts.Viewer.Models;
using System.Collections.Generic;
using System.Text.Encodings.Web;
using System.Linq;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;
using Assets.Scripts.Common.UI;
using System;


using Assets.Scripts.Viewer;
using Assets.Scripts.Viewer.Behaviors;
namespace Assets.Scripts.Navigation.Behaviors
{
    public class ViewToggleBehavior : ViewerStateBehavior
    {
        //[SerializeField]
        //private int selectedView;

        //[SerializeField]
        //public BetterToggleGroup orientationGroup;
        //public ViewToggleBehavior()
        //{
        //    this.RegisterStateChangeHandler(state => state.DualView, this.OnDualViewChanged);
        //}

        //protected override void OnEnable()
        //{
        //    base.OnEnable();

        //    var dualView = this.GetStateProperty(state => state.DualView).Value;

        //    this.OnDualViewChanged(dualView, dualView);
        //    this.orientationGroup = gameObject.GetComponentInChildren<BetterToggleGroup>();
        //    if (this.orientationGroup)
        //    {
        //        this.orientationGroup.OnChange += OrientationGroup_OnChange;
        //    }

        //}

        //private void OrientationGroup_OnChange(Toggle newActive)
        //{
        //    ViewButtonBehavior orientationBeh = newActive.GetComponent<ViewButtonBehavior>();
        //    if (orientationBeh != null)
        //    {
        //        int id = orientationBeh.GetViewId();

        //        this.SetSelectedView(id);
        //    }
        //    else
        //    {
        //        Debug.LogWarning("Missing object ViewButtonBehavior");
        //    }
        //}



        //private void OnDualViewChanged(bool newDualView, bool oldDualView)
        //{
        //    HashSet<int> viewsToEnable = new HashSet<int>(new int[] { 1 });
        //    if (newDualView)
        //    {
        //        viewsToEnable.Add(2);
        //    }

        //    var viewBehaviors = this.GetComponentsInChildren<ViewButtonBehavior>(true);
        //    if (viewBehaviors != null)
        //    {
        //        foreach (ViewButtonBehavior beh in viewBehaviors)
        //        {
        //            int id = beh.GetViewId();
        //            if (id != 0)
        //            {
        //                beh.gameObject.SetActive(viewsToEnable.Contains(id));

        //            }
        //            else if (beh.tag != null)
        //            {
        //                if (beh.tag == ViewTags.GetTags(ViewTagsTypes.view1))
        //                {
        //                    beh.SetToggleState(true);
        //                    id = 1;
        //                    this.SetSelectedView(1);
        //                }
        //                else if (beh.tag == ViewTags.GetTags(ViewTagsTypes.view2))
        //                {
        //                    id = 2;
        //                }
        //                beh.SetViewId(id);
        //                beh.gameObject.SetActive(viewsToEnable.Contains(id));
        //            }


        //        }

        //    }


        //}

        //public void SetSelectedView(int id)
        //{
        //    this.selectedView = id;
        //    var navoptions = gameObject.GetComponentsInChildren<NavigationalBehavior>(true);
        //    foreach (NavigationalBehavior option in navoptions)
        //    {
        //        if (option != null)
        //        {
        //            option.SetViewId(id);
        //        }

        //    }

        //}

        //public int GetSelectedView(int id)
        //{
        //    return this.selectedView;
        //}
    }
}
