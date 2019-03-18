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
    private List<(Vector2, Vector2)> lineSegments;
    private BoxCollider collider;

    void Start()
    {
        collider = GetComponent<BoxCollider>();
        lineSegments = GetLineSegments();
        car = gameObject.GetComponent<Rigidbody>();
        var wheelColliders = gameObject.GetComponentsInChildren<WheelCollider>();
        var wheels = gameObject.transform.Find("Wheels");

        frontDriverT = wheels.Find("FrontDriver");
        frontPassengerT = wheels.Find("FrontPassenger");
        rearDriverT = wheels.Find("RearDriver");
        rearPassengerT = wheels.Find("RearPassenger");

        frontDriverW = wheelColliders.First(q => q.name == "FrontDriver");
        frontPassengerW = wheelColliders.First(q => q.name == "FrontPassenger");
        rearDriverW = wheelColliders.First(q => q.name == "RearDriver");
        rearPassengerW = wheelColliders.First(q => q.name == "RearPassenger");
    }

	private void FixedUpdate()
	{
		Steer();
		Accelerate();
		UpdateWheelPoses();
    }

	private float m_horizontalInput;
	private float m_verticalInput;
	private float m_steeringAngle;

	public float maxSteerAngle = 30;
	public float motorForce = 200;
    public float BrakeForce = 200;
    public PathCreator pathCreator;

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        var carPosition = new Vector2(car.position.x, car.position.z);

        var distance = lineSegments.Min(q => DistancePointLine(carPosition, q.Item1, q.Item2));
        var velocity = Vector3.Dot(car.velocity, gameObject.transform.forward);

        m_horizontalInput = vectorAction[1];
        m_verticalInput = vectorAction[0];

        if (IsOnGrass())
        {
            Done();
            AddReward(-10);
        }
        else
        {
            AddReward(velocity / (1 + distance));
        }

        base.AgentAction(vectorAction, textAction);
    }

    public override void AgentReset()
    {
        car.velocity = Vector3.zero;
        var idx = Random.Range(0, pathCreator.EditorData.bezierPath.NumAnchorPoints - 1);
        var pointsInSegment = pathCreator.EditorData.bezierPath.GetPointsInSegment(idx);

        transform.position = new Vector3(pointsInSegment[0].x, 0.01f, pointsInSegment[0].z);
        transform.LookAt(pointsInSegment[1]);
    }

    private bool IsOnGrass()
    {
        var boundPoint1 = collider.bounds.min;
        var boundPoint2 = collider.bounds.max;
        var boundPoint4 = new Vector3(boundPoint1.x, boundPoint2.y, boundPoint1.z);
        var boundPoint6 = new Vector3(boundPoint1.x, boundPoint2.y, boundPoint2.z);
        var boundPoint8 = new Vector3(boundPoint2.x, boundPoint2.y, boundPoint1.z);

        var rays = new List<Ray>()
        {
            new Ray(boundPoint6, Vector3.down),
            new Ray(boundPoint2, Vector3.down),
            new Ray(boundPoint8, Vector3.down),
            new Ray(boundPoint4, Vector3.down)
        };

        return rays
            .Any(q => Physics.Raycast(q, out RaycastHit castInfo, 9) && castInfo.collider.name == "Grass");

    }

    private static float DistancePointLine(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        return Vector3.Magnitude(ProjectPointLine(point, lineStart, lineEnd) - point);
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
