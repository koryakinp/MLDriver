using UnityEngine;
using System.Collections;

public class WheelPhysicMaterial : MonoBehaviour
{
    public float BrakeOnGrass = 50f;
    private WheelCollider[] wheels;

    void Start()
    {
        wheels = gameObject.GetComponentsInChildren<WheelCollider>();
    }

    void FixedUpdate()
    {
        WheelHit hit;

        for (int i = 0; i < wheels.Length; i++)
        {
            var wheel = wheels[i];

            if (wheel.GetGroundHit(out hit))
            {
                wheel.brakeTorque = hit.collider.material.name == "Grass Surface (Instance)" ? BrakeOnGrass : 0;
            }
        }
    }
}