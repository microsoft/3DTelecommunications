using UnityEngine;
using System;

[Serializable]
public class DepthDilateProfile : ScriptableObject
{
    public Configuration HoloportationConfig;

    public int DilateSteps;
  

    public int[] DepthSearchRadius;
    public int[] DepthSearchStep;

    public int[] DilateStepSizes;
}
