using System.Linq;
using UnityEngine;
using MLAgents;
using PathCreation;
using System.Collections.Generic;

public class DriverAgent : Agent
{
    private WheelCollider frontDriverW, frontPassengerW, rearDriverW, rearPassengerW;
    private Transform frontDriverT, frontPassengerT, rearDriverT, rearPassengerT;
    private Rigidbody car;
    private List<(Vector2, Vector2)> lineSegments;
    private int layer_mask;

    private float _turn;

    void Start()
    {
        layer_mask = LayerMask.GetMask("Surface");
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
        Accelerate();
		Steer();
        UpdateWheelPoses();
    }

	private float m_steeringAngle;

	public float maxSteerAngle = 30;
	public float MotorForce = 100;
    public PathCreator pathCreator;

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        var carPosition = new Vector2(car.position.x, car.position.z);

        var distance = lineSegments.Min(q => DistancePointLine(carPosition, q.Item1, q.Item2));
        var velocity = Vector3.Dot(car.velocity, gameObject.transform.forward);

        _turn = vectorAction[0];

        if (IsOnGrass())
        {
            Done();
            AddReward(-10);
        }
        else
        {
            var reward = velocity / (1 + distance);
            AddReward(reward);
        }
    }

    public override void AgentReset()
    {
        car.velocity = Vector3.zero;
        var idx = Random.Range(0, pathCreator.EditorData.bezierPath.NumAnchorPoints - 1);
        var pointsInSegment = pathCreator.EditorData.bezierPath.GetPointsInSegment(idx);

        transform.position = new Vector3(pointsInSegment[0].x, 0.01f, pointsInSegment[0].z);
        transform.LookAt(pointsInSegment[1]);

        frontDriverW.brakeTorque = 0;
        frontPassengerW.brakeTorque = 0;
        rearDriverW.brakeTorque = 0;
        rearPassengerW.brakeTorque = 0;
    }

    private bool IsOnGrass()
    {
        var rayStart = transform.position + new Vector3(0, 1, 0);
        var ray = new Ray(rayStart, Vector3.down);
        Physics.Raycast(ray, out RaycastHit castInfo2, 2, layer_mask);
        if(castInfo2.collider.name == "Grass")
        {
            frontDriverW.brakeTorque = float.MaxValue;
            frontPassengerW.brakeTorque = float.MaxValue;
            rearDriverW.brakeTorque = float.MaxValue;
            rearPassengerW.brakeTorque = float.MaxValue;
            m_steeringAngle = 0;
            return true;
        }

        return false;
    }

    private static float DistancePointLine(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
    {
        return Vector3.Magnitude(ProjectPointLine(point, lineStart, lineEnd) - point);
    }

    private void Steer()
    {
        if(_turn == 1)
        {
            m_steeringAngle+=2;
        }
        else if(_turn == 2)
        {
            m_steeringAngle-=2;
        }
        else
        {
            if(m_steeringAngle > 0)
            {
                m_steeringAngle-=2;
            }
            
            if(m_steeringAngle < 0)
            {
                m_steeringAngle+=2;
            }
        }

        if(m_steeringAngle > maxSteerAngle)
        {
            m_steeringAngle = maxSteerAngle;
        }

        if (m_steeringAngle < -maxSteerAngle)
        {
            m_steeringAngle = -maxSteerAngle;
        }

        frontDriverW.steerAngle = m_steeringAngle;
        frontPassengerW.steerAngle = m_steeringAngle;
    }

    private void Accelerate()
    {
        frontDriverW.motorTorque = MotorForce;
        frontPassengerW.motorTorque = MotorForce;
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
