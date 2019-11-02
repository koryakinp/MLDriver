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
    private List<(Vector3, Vector3)> lineSegments;
    private Camera _camera;
    private int layer_mask;
    private Vector3 _prevIntersect = Vector3.zero;
    private DriverAcademy _academy;
    private float _reward;
    private float _penalty;
    private float _turn;

    void Start()
    {
        _academy = GameObject.FindObjectOfType<DriverAcademy>();
        _camera = Camera.main;

        layer_mask = LayerMask.GetMask("Surface");
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
    public PathCreator pathCreator;

    public override void AgentAction(float[] vectorAction, string textAction)
    {
        Accelerate();
		Steer();
        UpdateWheelPoses();

        _turn = vectorAction[0];

        if (IsOnGrass())
        {
            Done();
            AddReward(_penalty);
        }
        else
        {
            AddReward(_reward);
            UpdateCamera();
        }
    }

    public override void AgentReset()
    {
        _reward = _academy.resetParameters["reward"];
        _penalty = _academy.resetParameters["penalty"];
        _prevIntersect = Vector3.zero;
        var idx = UnityEngine.Random.Range(0, pathCreator.EditorData.bezierPath.NumAnchorPoints - 1);
        var pointsInSegment = pathCreator.EditorData.bezierPath.GetPointsInSegment(idx);

        transform.position = new Vector3(pointsInSegment[0].x, 0.01f, pointsInSegment[0].z);
        transform.LookAt(pointsInSegment[1]);
        car.velocity = car.transform.forward;

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
}
