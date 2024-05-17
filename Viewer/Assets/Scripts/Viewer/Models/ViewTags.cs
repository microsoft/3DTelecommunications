using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;



namespace Assets.Scripts.Viewer.Models
{

    public enum ViewTagsTypes
    {
        view1,
        view2,
    }

    public static class ViewTags
    {
        public static string GetTags(ViewTagsTypes type)
        {
            switch (type)
            {
                case ViewTagsTypes.view1:
                    return "View1";
                case ViewTagsTypes.view2:
                    // Blue highlight
                    return "View2";
                default:
                    return "View1";
            }
        }

    }

}






