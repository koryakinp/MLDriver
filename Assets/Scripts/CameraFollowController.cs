using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollowController : MonoBehaviour {


    private void FixedUpdate()
    {
        transform.rotation = Quaternion.Euler(90, objectToFollow.rotation.eulerAngles.y + 90, 90);
        transform.position = objectToFollow.position + objectToFollow.up * cameraHeight;
    }

    public Transform objectToFollow;
    public float cameraHeight = 10;
}
