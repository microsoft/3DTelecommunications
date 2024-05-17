using Assets.Scripts.Viewer;
using Assets.Scripts.Viewer.Models;
using System;
using UnityEngine;
using UnityEngine.UI;

public class Ticker : ViewerStateBehavior
{
    private DateTime startTime;

    protected override void OnEnable()
    {
        RegisterStateChangeHandler(state => state.OverallConnectionStatus, OnConnectionStatusChange);

        base.OnEnable();
    }

    // Update is called once per frame
    void Update()
    {
        var text = this.GetComponent<Text>();
        if (text != null)
        {
            if (this.GetStateProperty(state => state.OverallConnectionStatus).Value == ConnectionStatus.Connected)
            {
                text.text = (DateTime.Now.Subtract(this.startTime)).ToString(@"mm\:ss");
            } else
            {
                text.text = "";
            }
        }
    }

    private void OnConnectionStatusChange(ConnectionStatus cs, ConnectionStatus oldCs)
    {
        if (cs == ConnectionStatus.Connected && oldCs != ConnectionStatus.Connected) 
        {
            this.startTime = DateTime.Now;
        }
    }
}
