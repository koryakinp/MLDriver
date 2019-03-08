using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollowController : MonoBehaviour
{
    private void FixedUpdate()
    {
        Vector3 dir = objectToFollow.transform.forward;

        transform.rotation = Quaternion.Euler(90, objectToFollow.rotation.eulerAngles.y + 90, 90);
        transform.position = new Vector3(objectToFollow.position.x, cameraHeight, objectToFollow.position.z);
        transform.position += dir * distance;
    }

    public Transform objectToFollow;
    public float cameraHeight = 10;
    public float distance = 3;
}
