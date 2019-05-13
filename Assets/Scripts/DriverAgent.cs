using System.Linq;
using UnityEngine;
using MLAgents;
using PathCreation;
using System.Collections.Generic;
using System;

public class DriverAgent : Agent
{
    private WheelCollider frontDriverW, frontPassengerW, rearDriverW, rearPassengerW;
    private Transform frontDriverT, frontPassengerT, rearDriverT, rearPassengerT;
    private Rigidbody car;
    private List<(Vector3, Vector3)> lineSegments;
    private Camera _camera;
    private int layer_mask;
    private Vector3 _prevIntersect = Vector3.zero;

    private float _turn;

    void Start()
    {
        _camera = Camera.main;
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

        UpdateCamera();

        for (int i = 0; i < lineSegments.Count; i++)
        {
            Debug.DrawLine(lineSegments[i].Item1, lineSegments[i].Item2, Color.red, 100);
        }
    }

    private void UpdateCamera()
    {
        Vector3 dir = car.transform.forward;
        _camera.transform.rotation = Quaternion.Euler(90, car.rotation.eulerAngles.y + 90, 90);
        _camera.transform.position = new Vector3(car.position.x, cameraHeight, car.position.z);
        _camera.transform.position += dir * distance;
    }

	private float m_steeringAngle;
    public float cameraHeight = 10;
    public float distance = 3;
	public float maxSteerAngle = 30;
	public float MotorForce = 100;
    public float DistanceRewardFactor = 10;
    public float RewardPenalty = -10;
    public PathCreator pathCreator;

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        var rm = ComputeRewardMetric(car.position);

        Accelerate();
		Steer();
        UpdateWheelPoses();

        _turn = vectorAction[0];

        if (IsOnGrass())
        {
            Done();
            AddReward(RewardPenalty);
        }
        else
        {
            var reward = rm.DistanceFromPrevPosition * DistanceRewardFactor;
            AddReward(reward);
            UpdateCamera();
        }
    }

    public override void AgentReset()
    {
        _prevIntersect = Vector3.zero;
        car.velocity = Vector3.zero;
        var idx = UnityEngine.Random.Range(0, pathCreator.EditorData.bezierPath.NumAnchorPoints - 1);
        var pointsInSegment = pathCreator.EditorData.bezierPath.GetPointsInSegment(idx);

        transform.position = new Vector3(pointsInSegment[0].x, 0.01f, pointsInSegment[0].z);
        transform.LookAt(pointsInSegment[1]);

        frontDriverW.brakeTorque = 0;
        frontPassengerW.brakeTorque = 0;
        rearDriverW.brakeTorque = 0;
        rearPassengerW.brakeTorque = 0;
        UpdateCamera();
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

    private Vector3 GetIntersect(Vector3 carPosition)
    {
        carPosition = Vector3.ProjectOnPlane(carPosition, Vector3.down);

        var tempDistance = float.MaxValue;
        var tempIntersect = Vector3.zero;

        foreach (var lineSegment in lineSegments)
        {
            Vector3 curIntersect = ProjectPointLine(carPosition, lineSegment.Item1, lineSegment.Item2);
            var distance = Vector3.Magnitude(curIntersect - carPosition);

            if(distance < tempDistance)
            {
                tempDistance = distance;
                tempIntersect = curIntersect;
            }
        }

        return tempIntersect;
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

    private RewardMetric ComputeRewardMetric(Vector3 carPosition)
    {
        var curIntersect = GetIntersect(car.position);

        var distanceTraveled = _prevIntersect != Vector3.zero
            ? Vector3.Magnitude(_prevIntersect - curIntersect)
            : 0;

        var distanceFromCenterline = Vector3.Magnitude(curIntersect - car.position);

        _prevIntersect = new Vector3(curIntersect.x, curIntersect.y, curIntersect.z);
        var velocity = Vector3.Dot(car.velocity, gameObject.transform.forward);

        return new RewardMetric(distanceFromCenterline, distanceTraveled, velocity);
    }

    private List<(Vector3, Vector3)> GetLineSegments()
    {
        var lines = new List<(Vector3, Vector3)>();

        for (int i = 0; i < pathCreator.path.NumVertices; i++)
        {
            var start = pathCreator.path.vertices[i];
            
            var end = i == pathCreator.path.NumVertices - 1 
                ? pathCreator.path.vertices[0] 
                : pathCreator.path.vertices[i + 1];

            start = Vector3.ProjectOnPlane(start, Vector3.down);
            end = Vector3.ProjectOnPlane(end, Vector3.down);

            lines.Add((start, end));
        }

        return lines;
    }

    private class RewardMetric
    {
        public readonly float DistanceFromCenterline;
        public readonly float DistanceFromPrevPosition;
        public readonly float Speed;

        public RewardMetric(
            float distanceFromCenterline,
            float distanceFromPrevPosition,
            float speed)
        {
            DistanceFromCenterline = distanceFromCenterline;
            DistanceFromPrevPosition = distanceFromPrevPosition;
            Speed = speed;
        }
    }

}
