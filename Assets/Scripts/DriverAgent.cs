using System.Linq;
using UnityEngine;
using MLAgents;
using PathCreation;
using UnityEngine.UI;
using System.Collections.Generic;

public class DriverAgent : Agent
{
    private WheelCollider frontDriverW, frontPassengerW, rearDriverW, rearPassengerW;
    private Transform frontDriverT, frontPassengerT, rearDriverT, rearPassengerT;
    private Rigidbody car;
    private Text uitext1;
    private Text uitext2;
    private List<(Vector2, Vector2)> lineSegments;

    void Start()
    {
        lineSegments = GetLineSegments();
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
        var carPosition = new Vector2(car.position.x, car.position.z);

        var distance = lineSegments.Min(q => DistancePointLine(carPosition, q.Item1, q.Item2));
        var velocity = Vector3.Dot(car.velocity, gameObject.transform.forward);

        uitext1.text = $"Speed: {velocity.ToString("#.##")}";
        uitext2.text = $"Offset: {distance.ToString("#.##")}";

        m_horizontalInput = vectorAction[1];
        m_verticalInput = vectorAction[0];

        base.AgentAction(vectorAction, textAction);
    }

    public static float DistancePointLine(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        return Vector3.Magnitude(ProjectPointLine(point, lineStart, lineEnd) - point);
    }

    private static Vector3 ProjectPointLine(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        Vector3 relativePoint = point - lineStart;
        Vector3 lineDirection = lineEnd - lineStart;
        float length = lineDirection.magnitude;
        Vector3 normalizedLineDirection = lineDirection;
        if (length > .000001f)
            normalizedLineDirection /= length;

        float dot = Vector3.Dot(normalizedLineDirection, relativePoint);
        dot = Mathf.Clamp(dot, 0.0F, length);

        return lineStart + normalizedLineDirection * dot;
    }

    private List<(Vector2, Vector2)> GetLineSegments()
    {
        var lines = new List<(Vector2, Vector2)>();

        for (int i = 0; i < pathCreator.path.NumVertices; i++)
        {
            var start = pathCreator.path.vertices[i];
            var end = i == pathCreator.path.NumVertices - 1 
                ? pathCreator.path.vertices[0] 
                : pathCreator.path.vertices[i + 1];
            lines.Add((new Vector2(start.x, start.z), new Vector2(end.x, end.z)));
        }

        return lines;
    }
}
