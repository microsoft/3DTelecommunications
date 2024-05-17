using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


[Serializable]
public class MessageType
{
    public string TypeName;
    public Color TypeColor;
    public Sprite TypeIcon;
}
public class DialogManager : MonoBehaviour
{
    //threadsafe SINGLETON pattern
    protected static DialogManager instance;
    // Explicit static constructor to tell C# compiler
    // not to mark type as beforefieldinit
    static DialogManager()
    {
        
    }

    private DialogManager()
    {
        
    }

    public static DialogManager Instance
    {
        get
        {
            return instance;
        }
    }

    [Header("Types")]
    public MessageType ErrorMessage;
    public MessageType NormalMessage;
    public MessageType WarningMessage;
    [Header("UI Link")]
    public Text messageText;
    public Image messageIcon;

    public int textCountlimit;
    private string latestMessage;
    private MessageType latestMessageType;
    // Start is called before the first frame update
    void Start()
    {
        if (instance == null)
        {
            instance = this;
            instance.latestMessage = "";
            instance.UpdateMessage("Started.", instance.NormalMessage);            
        }       
    }

    // Update is called once per frame
    void Update()
    {
        if(latestMessage.Length > 0)
        {
            messageText.text = latestMessage;
            messageText.color = latestMessageType.TypeColor;
            messageIcon.color = latestMessageType.TypeColor;
            messageIcon.sprite = latestMessageType.TypeIcon;

            latestMessage = "";
        }
    }

    //can be called from non-mainThread
    public void UpdateMessage(string message, MessageType messageType)
    {
        latestMessage = message;
        if (latestMessage.Length > textCountlimit)
            latestMessage = latestMessage.Substring(0, textCountlimit);
        latestMessageType = messageType;
    }
}
