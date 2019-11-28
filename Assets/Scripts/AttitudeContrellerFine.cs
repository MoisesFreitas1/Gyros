using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AttitudeContrellerFine : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Input.gyro.enabled = true;

    }

    // Update is called once per frame
    void Update()
    {
        Quaternion q = Input.gyro.attitude;
        q = GyroToUnity(q);
        transform.rotation = q;
    }

    private static Quaternion GyroToUnity(Quaternion q)
    {
        return new Quaternion(q.x, q.z, q.y, -q.w);
    }
}
