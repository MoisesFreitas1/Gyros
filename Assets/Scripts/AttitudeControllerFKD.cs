using System.Collections;
using Accord.Math;
using UnityEngine;

public class AttitudeControllerFKD : MonoBehaviour
{
    public float p, q, r;
    public float ax, ay, az;
    public float mx, my, mz;
    public float mnx, mny, mnz;
    public float bx, by, bz;
    public float phi, theta, psi;
    public float dt;

    private Quaternion qt;
    private float time;
    private int ntimes;
    private float sBiasX, sBiasY, sBiasZ;
    private float g = -9.80665f;
    private double[,] A;
    private double[,] B;
    private double[,] H;
    private double[,] P;
    private double[,] Q;
    private double[,] R;
    private double[,] x;
    private double[,] x_til;
    private double[,] z;
    private double[,] u;

    private bool biasOk;

    private void Awake()
    {
        Input.compass.enabled = true;
    }

    void Start()
    {
        biasOk = false;
        time = 0f;
        ntimes = 0;
        sBiasX = 0f;
        sBiasY = 0f;
        sBiasZ = 0f;
        //Input.gyro.enabled = true;
        //Input.compass.enabled = true;

        p = Input.gyro.rotationRate.x;
        q = Input.gyro.rotationRate.y;
        r = Input.gyro.rotationRate.z;

        ax = Input.acceleration.x;
        ay = Input.acceleration.y;
        az = Input.acceleration.z;

        mx = Input.compass.rawVector.x;
        my = Input.compass.rawVector.y;
        mz = Input.compass.rawVector.z;

        theta = (180 / Mathf.PI) * Mathf.Asin(-ax / g);
        phi = (180 / Mathf.PI) * Mathf.Atan2(ay, az);

        mnx = Mathf.Cos(theta) * mx + Mathf.Sin(phi) * Mathf.Sin(theta) * my - Mathf.Cos(phi) * Mathf.Sin(theta) * mz;
        mny = Mathf.Cos(phi) * my + Mathf.Sin(phi) * mz;

        psi = (180 / Mathf.PI) * Mathf.Atan2(mny, mnx);

        Debug.Log("theta: " + theta + "\nphi:" + phi + "\npsi: " + psi);

        // inicializacao do bias 
        bx = p;
        by = q;
        bz = r;

        qt = Quaternion.Euler(phi, theta, psi);

        x_til = new double[,]
        {
            {qt.w},
            {qt.x},
            {qt.y},
            {qt.z},
            {bx},
            {by},
            {bz}
        };

        u = new double[,]
        {
            {p},
            {q},
            {r}
        };

        P = new double[,]
        {
            {1e-15, 0, 0, 0, 0, 0, 0},
            {0, 1e-15, 0, 0, 0, 0, 0},
            {0, 0, 1e-15, 0, 0, 0, 0},
            {0, 0, 0, 1e-15, 0, 0, 0},
            {0, 0, 0, 0, 1e-15, 0, 0},
            {0, 0, 0, 0, 0, 1e-15, 0},
            {0, 0, 0, 0, 0, 0, 1e-15}
        };

        Q = new double[,]
        {
            {1e-15, 0, 0, 0, 0, 0, 0},
            {0, 1e-15, 0, 0, 0, 0, 0},
            {0, 0, 1e-15, 0, 0, 0, 0},
            {0, 0, 0, 1e-15, 0, 0, 0},
            {0, 0, 0, 0, 1e-15, 0, 0},
            {0, 0, 0, 0, 0, 1e-15, 0},
            {0, 0, 0, 0, 0, 0, 1e-15}
        };

        R = new double[,]
        {
            {0.0015, 0, 0, 0},
            {0, 0.0020, 0, 0},
            {0, 0, 0.005, 0},
            {0, 0, 0, 1}
        };
    }

    void Update()
    {
        if (time < 30)
        {
            time = time + Time.fixedDeltaTime;
            ntimes = ntimes + 1;
            sBiasX = sBiasX + Input.gyro.rotationRate.x;
            sBiasY = sBiasY + Input.gyro.rotationRate.y;
            sBiasZ = sBiasZ + Input.gyro.rotationRate.z;
        }
        else if (time >= 30)
        {
            if (!biasOk)
            {
                biasOk = true;
                bx = sBiasX / ntimes;
                by = sBiasY / ntimes;
                bz = sBiasZ / ntimes;

                x_til = new double[,]
                {
                    {qt.w},
                    {qt.x},
                    {qt.y},
                    {qt.z},
                    {bx},
                    {by},
                    {bz}
                };

                x = new double[,]
                {
                    {qt.w},
                    {qt.x},
                    {qt.y},
                    {qt.z},
                    {bx},
                    {by},
                    {bz}
                };
            }


            dt = Time.deltaTime;
            A = new double[,]
            {
                {1, 0, 0, 0, dt*qt.x/2, dt*qt.y/2, dt*qt.z/2},
                {0, 1, 0, 0, -dt*qt.w/2, dt*qt.x/2, -dt*qt.y/2},
                {0, 0, 1, 0, -dt*qt.z/2, -dt*qt.w/2, dt*qt.x/2},
                {0, 0, 0, 1, dt*qt.y/2, -dt*qt.x/2, -dt*qt.w/2},
                {0, 0, 0, 0, 1, 0, 0},
                {0, 0, 0, 0, 0, 1, 0},
                {0, 0, 0, 0, 0, 0, 1},
            };

            B = new double[,]
            {
                {dt*qt.x/2, dt*qt.y/2, dt*qt.z/2},
                {-dt*qt.w/2, dt*qt.x/2, -dt*qt.y/2},
                {-dt*qt.z/2, -dt*qt.w/2, dt*qt.x/2},
                {dt*qt.y/2, -dt*qt.x/2, -dt*qt.w/2},
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0},
            };

            H = new double[,]
            {
                {1, 0, 0, 0, 0, 0, 0},
                {0, 1, 0, 0, 0, 0, 0},
                {0, 0, 1, 0, 0, 0, 0},
                {0, 0, 0, 1, 0, 0, 0}
            };

            // Novas medidas
            p = Input.gyro.rotationRate.x;
            q = Input.gyro.rotationRate.y;
            r = Input.gyro.rotationRate.z;

            ax = Input.acceleration.x;
            ay = Input.acceleration.y;
            az = Input.acceleration.z;
            
            mx = Input.compass.rawVector.x;
            my = Input.compass.rawVector.y;
            mz = Input.compass.rawVector.z;

            Debug.Log(Input.compass.rawVector.ToString());

            theta = (180 / Mathf.PI) * Mathf.Asin(-ax / g);
            phi = (180 / Mathf.PI) * Mathf.Atan2(ay, az);

            mnx = Mathf.Cos(theta) * mx + Mathf.Sin(phi) * Mathf.Sin(theta) * my - Mathf.Cos(phi) * Mathf.Sin(theta) * mz;
            mny = Mathf.Cos(phi) * my + Mathf.Sin(phi) * mz;

            psi = (180 / Mathf.PI) * Mathf.Atan2(mny, mnx);
            Debug.Log("theta: " + theta + "\nphi:" + phi + "\npsi: " + psi);

            qt = Quaternion.Euler(phi, theta, psi);

            u = new double[,]
            {
                {p},
                {q},
                {r}
            };

            x_til = new double[,]
            {
                {qt.w},
                {qt.x},
                {qt.y},
                {qt.z},
                {bx},
                {by},
                {bz}
            };


            // Filtro de Kalman Direto
            double[,] x_1 = A.Dot(x); // 7x7 . 7x1 = 7x1
            double[,] x_2 = B.Dot(u); // 7x3 .  3x1 = 7x1
            x = x_1.Add(x_2);  // 7x1 + 7x1 = 7x1

            z = H.Multiply(x_til); // 4x7 . 7x1 = 4x1

            double[,] P_0 = A.Transpose(); // 7x7 = 7x7
            double[,] P_1 = P.Dot(P_0); // 7x7 . 7x7 = 7x7
            double[,] P_2 = A.Dot(P_1); // 7x7 . 7x7 = 7x7
            P = P_2.Add(Q); // 7x7 + 7x7 = 7x7

            double[,] k_0 = H.Transpose();
            double[,] k_1 = P.Dot(k_0); // 7x7 . 7.4 = 7x4
            double[,] k_2 = H.Dot(k_1); // 4x7 . 7x4 = 4x4
            double[,] k_3 = k_2.Add(R); // 4x4 + 4x4 = 4x4
            double[,] k_4 = k_3.Inverse(); // 4x4
            double[,] k_5 = P.Dot(H.Transpose()); // 7x7 . 7x4 = 7x4
            double[,] K = k_5.Dot(k_4); // 7x4 . 4x4 = 7x4

            double[,] x_3 = H.Dot(x); // 4x7 . 7x1 = 4x1
            double[,] x_4 = z.Subtract(x_3); // 4x1 - 4x1 = 4x1
            double[,] x_5 = K.Dot(x_4); // 7x4 . 4x1 = 7x1
            x = x.Add(x_5); // 7x1 + 7x1 = 7x1

            double[,] P_3 = H.Dot(P); // 4x7 . 7x7 = 4x7
            double[,] P_4 = K.Dot(P_3); // 7x4 . 4x7 = 7x7
            P = P.Subtract(P_4); // 7x7 - 7x7 = 7x7

            //double[,] R_1 = R.Add(K); // 4x4 + 7x4 = Errado!!!
            //double[,] R_2 = R_1.Inverse(); 
            //double[,] R_3 = R.Multiply(R_2);
            //R = Matrix.Identity(4).Add(R_3);

            Quaternion x_ = new Quaternion
            {
                w = (float)x[0, 0],
                x = (float)x[1, 0],
                y = (float)x[2, 0],
                z = (float)x[3, 0]
            };

            transform.rotation = x_;
        }
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

        return q;
    }
}
