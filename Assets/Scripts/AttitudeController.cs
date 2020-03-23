using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AttitudeController : MonoBehaviour
{
    //private float phi0, theta0, psi0;
    public float p, q, r;
    private float phi, theta, psi;
    private float time;
    // Start is called before the first frame update
    void Start()
    {
        time = 0f;
        Input.gyro.enabled = true;
        theta = Input.gyro.attitude.eulerAngles.z;
        phi = Input.gyro.attitude.eulerAngles.x;
        psi = Input.gyro.attitude.eulerAngles.y;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        p = -Input.gyro.rotationRate.z;
        q = -Input.gyro.rotationRate.x;
        r = -Input.gyro.rotationRate.y;

        Vector3 Wn = Body2Station(theta, phi, p, q, r);

        theta = theta + Wn[1] * Time.fixedDeltaTime;
        phi = phi + Wn[0] * Time.fixedDeltaTime;
        psi = psi + Wn[2] * Time.fixedDeltaTime;

        transform.rotation = Euler2Quaternion(theta, phi, psi);
    }

    private Vector3 Body2Station(float theta, float phi, float p, float q, float r)
    {
        float a1 = p + q * (Mathf.Sin(phi) * Mathf.Tan(theta)) + r * (Mathf.Cos(phi) * Mathf.Tan(theta));
        float a2 = q * Mathf.Cos(phi) + r * Mathf.Sin(phi);
        float a3 = q * (Mathf.Sin(phi) / Mathf.Cos(theta)) + r * (Mathf.Cos(phi) / Mathf.Cos(theta));

        return new Vector3(a1, a2, a3);

    }

    private Quaternion Euler2Quaternion(float roll, float pitch, float yaw)
    {
        float cy = Mathf.Cos(yaw * 0.5f);
        float sy = Mathf.Sin(yaw * 0.5f);
        float cp = Mathf.Cos(pitch * 0.5f);
        float sp = Mathf.Sin(pitch * 0.5f);
        float cr = Mathf.Cos(roll * 0.5f);
        float sr = Mathf.Sin(roll * 0.5f);

        Quaternion q;
        q.w = cy * cp * cr + sy * sp * sr;
        q.x = cy * cp * sr - sy * sp * cr;
        q.y = sy * cp * sr + cy * sp * cr;
        q.z = sy * cp * cr - cy * sp * sr;

        //GyroToUnity(q);

        return q;
    }

    private static Quaternion GyroToUnity(Quaternion q)
    {
        return new Quaternion(q.z, q.x, q.y, -q.w);
    }
}
