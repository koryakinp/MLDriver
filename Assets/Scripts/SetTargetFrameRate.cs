using System;
using UnityEngine;
 
 public class SetTargetFrameRate : MonoBehaviour 
 {
     public int targetFrameRate = 10;
 
     private void Start()
     {
         QualitySettings.vSyncCount = 0;
         Application.targetFrameRate = targetFrameRate;
         Debug.Log("Start");
     }
 }