using Assets.Scripts.Viewer.Models;
using Assets.Scripts.Viewer.State.Actions;
using UnityEngine;
using UnityEngine.UI;
using Assets.Scripts.Common.Extensions;
using SimpleFileBrowser;


namespace Assets.Scripts.Viewer.Behaviors
{
    public class ExportPanelBehavior : ViewerStateBehavior
    {
        [SerializeField]
        private Text userFolderText;


        [SerializeField]
        private Button cancelButton;

        [SerializeField]
        private Button editButton;

        private string defaultFolder;


        protected override void OnEnable()
        {

            base.OnEnable();

            // if not in SerializeField, attempt to get in tree
            if (this.userFolderText == null)
            {
                this.userFolderText = this.GetNestedComponentByName<Text>("FolderInput");
            }
            if (this.cancelButton == null)
            {
                this.cancelButton = this.GetNestedComponentByName<Button>("CloseButton");
            }

            if (this.editButton == null)
            {
                this.editButton = this.GetNestedComponentByName<Button>("EditButton");
            }

            this.defaultFolder = SettingsManager.Instance.SnapshotsDirectory;
            this.SetInputText(defaultFolder);
            if (this.cancelButton != null)
            {
                cancelButton.onClick.AddListener(this.OnButtonCancel);
            }

            if (this.editButton != null)
            {
                editButton.onClick.AddListener(this.GetFileBrowser);
            }



            this.GetFileBrowser();
        }

        protected override void OnDisable()
        {
            base.OnDisable();

            FileBrowser.HideDialog();
        }

        public void GetFileBrowser()
        {
            FileBrowser.HideDialog();

            bool folderMode = true;
            var file = FileBrowser.ShowSaveDialog(onSuccess, onCancel, folderMode, false, null, "Save", "Save");

        }

        public void onSuccess(string[] paths)
        {
            if (paths != null && paths.Length >= 1)
            {
                string newPath = paths[0];
                this.SetInputText(newPath);
                SettingsManager.Instance.SnapshotsDirectory = newPath;
            }

        }

        public void onCancel()
        {


        }


        private void OnButtonCancel()
        {
            // Maybe cancel snapshot? 
            SettingsManager.Instance.SnapshotsDirectory = this.defaultFolder;
        }

        public void SetInputText(string text)
        {

            if (this.userFolderText != null)
            {
                this.userFolderText.text = text;
            }
        }



    }

}

