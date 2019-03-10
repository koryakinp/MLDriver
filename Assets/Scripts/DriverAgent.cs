﻿using System.Linq;
using UnityEngine;
using MLAgents;
using PathCreation;
using UnityEngine.UI;

public class DriverAgent : Agent
{
    private WheelCollider frontDriverW, frontPassengerW, rearDriverW, rearPassengerW;
    private Transform frontDriverT, frontPassengerT, rearDriverT, rearPassengerT;
    private Rigidbody car;
    private Text uitext1;
    private Text uitext2;

    void Start()
    {
        car = gameObject.GetComponent<Rigidbody>();
        var wheelColliders = gameObject.GetComponentsInChildren<WheelCollider>();
        var wheels = gameObject.transform.Find("Wheels");

        uitext1 = FindObjectsOfType<Text>().First(q => q.name == "Velocity");
        uitext2 = FindObjectsOfType<Text>().First(q => q.name == "Distance");

        frontDriverT = wheels.Find("FrontDriver");
        frontPassengerT = wheels.Find("FrontPassenger");
        rearDriverT = wheels.Find("RearDriver");
        rearPassengerT = wheels.Find("RearPassenger");

        frontDriverW = wheelColliders.First(q => q.name == "FrontDriver");
        frontPassengerW = wheelColliders.First(q => q.name == "FrontPassenger");
        rearDriverW = wheelColliders.First(q => q.name == "RearDriver");
        rearPassengerW = wheelColliders.First(q => q.name == "RearPassenger");
    }

	private void Steer()
	{
		m_steeringAngle = maxSteerAngle * m_horizontalInput;
		frontDriverW.steerAngle = m_steeringAngle;
		frontPassengerW.steerAngle = m_steeringAngle;
	}

	private void Accelerate()
	{
		frontDriverW.motorTorque = m_verticalInput * motorForce;
		frontPassengerW.motorTorque = m_verticalInput * motorForce;
	}

	private void UpdateWheelPoses()
	{
		UpdateWheelPose(frontDriverW, frontDriverT);
		UpdateWheelPose(frontPassengerW, frontPassengerT);
		UpdateWheelPose(rearDriverW, rearDriverT);
		UpdateWheelPose(rearPassengerW, rearPassengerT);
	}

	private void UpdateWheelPose(WheelCollider _collider, Transform _transform)
	{
		Vector3 _pos = _transform.position;
		Quaternion _quat = _transform.rotation;

		_collider.GetWorldPose(out _pos, out _quat);

		_transform.position = _pos;
		_transform.rotation = _quat;
	}

	private void FixedUpdate()
	{
		Steer();
		Accelerate();
		UpdateWheelPoses();
        BrakeOnGrass();
    }

	private float m_horizontalInput;
	private float m_verticalInput;
	private float m_steeringAngle;

	public float maxSteerAngle = 30;
	public float motorForce = 200;
    public float BrakeForce = 200;
    public PathCreator pathCreator;

    void BrakeOnGrass()
    {
        ApplyBrakeOnGrass(frontDriverW);
        ApplyBrakeOnGrass(frontPassengerW);
    }

    private void ApplyBrakeOnGrass(WheelCollider wheel)
    {
        WheelHit hit;
        if (wheel.GetGroundHit(out hit))
        {
            wheel.brakeTorque = hit.collider.material.name == "Grass Surface (Instance)" ? BrakeForce : 0;
        }
    }

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        var carPosition = new Vector3(car.position.x, 0, car.position.z);

        var distance = pathCreator.path.vertices.Min(q => Vector3.Distance(q, carPosition));
        var velocity = Vector3.Dot(car.velocity, gameObject.transform.forward);

        uitext1.text = $"Velocity: {velocity.ToString("#.##")}";
        uitext2.text = $"Distance: {distance.ToString("#.##")}";

        m_horizontalInput = vectorAction[1];
        m_verticalInput = vectorAction[0];

        base.AgentAction(vectorAction, textAction);
    }

    
}
