using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TorsionalVibrationSimulator : MonoBehaviour
{
    public float theta_i, theta_dot_i, I_ş, J_p, J_ş, J_t, k_b, w_n_b, Gş = 75e9f;

    public bool Solve, StartSimulating;
    public float deltaT;
    public int iterationCount;

    public GameObject ShaftN, MassN, ShaftA, MassA;
    public float pi => Mathf.PI;
    public float dş => GameObject.Find("Shaft").transform.localScale.x;
    public float Lş => GameObject.Find("Shaft").transform.localScale.y * 2;
    public float dp => GameObject.Find("Mass").transform.localScale.x;
    public float Lp => GameObject.Find("Mass").transform.localScale.y * 2;
    public float Vş => pi * dş * dş * Lş / 4;
    public float Vp => pi * dp * dp * Lp / 4;

    void Start()
    {
        ShaftN = GameObject.Find("Shaft");
        MassN = GameObject.Find("Mass");
        ShaftA = GameObject.Find("ShaftA");
        MassA = GameObject.Find("MassA");
        StartCoroutine(OnStarted());
    }

    private IEnumerator OnStarted()
    {
        while (true)
        {
            Application.targetFrameRate = /*(int)Mathf.Round(FPScount)*/33;
            QualitySettings.vSyncCount = 0;
            yield return new WaitForSeconds(0.25f);
        }
    }


    public int i_counter = 0;
    void FixedUpdate()
    {
        if (Solve)
        {
            Solve = false;
            Calculate();
            i_counter = 0;
        }
        if (StartSimulating && i_counter < _theta.Count - 1)
        {
            i_counter++;
            MassN.transform.eulerAngles = new Vector3(_theta[i_counter], MassN.transform.eulerAngles.y, 
                MassN.transform.eulerAngles.z);
            MassA.transform.eulerAngles = new Vector3(_thetaA[i_counter], MassA.transform.eulerAngles.y,
                MassA.transform.eulerAngles.z);
        }
        else
        {
            StartSimulating = false;
        }
    }

    public void Calculate()
    {
        I_ş = pi * pow(dş, 4) / 32;
        k_b = Gş * I_ş / Lş;
        J_p = Vp * dp * dp / 8;
        J_ş = Vş * dş * dş / 8;
        J_t = J_p + J_ş;
        w_n_b = sqrt(k_b / J_t);

        _time.Clear();
        _theta.Clear();

        RungeKuttaSolverSecondDegreeODE(0, theta_i * deg2rad, theta_dot_i, deltaT, iterationCount);
    }

    /*  ω_(n, b) = sqrt(k_b / j_b)  [rad/s]
        k_b = (G * I_ş) / L  [N*m/rad]
        I_ş = (π * d^4) / 32  [m^4]
        J = m * r^2 / 2
    */

    public List<float> _time, _theta, _thetaA;

    public void RungeKuttaSolverSecondDegreeODE(float t_init, float theta_init, float theta_dot_init, 
        float h, float iterationCount)
    {
        for (int i = 0; i < iterationCount; i++)
        {
            float k1 = h * f(t_init, theta_init, theta_dot_init);
            float l1 = h * g(t_init, theta_init, theta_dot_init);

            float k2 = h * f(t_init + h / 2, theta_init + k1 / 2, theta_dot_init + l1 / 2);
            float l2 = h * g(t_init + h / 2, theta_init + k1 / 2, theta_dot_init + l1 / 2);

            float k3 = h * f(t_init + h / 2, theta_init + k2 / 2, theta_dot_init + l2 / 2);
            float l3 = h * g(t_init + h / 2, theta_init + k2 / 2, theta_dot_init + l2 / 2);

            float k4 = h * f(t_init + h, theta_init + k3, theta_dot_init + l3);
            float l4 = h * g(t_init + h, theta_init + k3, theta_dot_init + l3);

            theta_init += (k1 + 2 * k2 + 2 * k3 + k4) / 6;
            theta_dot_init += (l1 + 2 * l2 + 2 * l3 + l4) / 6;

            t_init += h;

            _time.Add(t_init);
            _theta.Add(theta_init * rad2deg);
            _thetaA.Add((theta_i * deg2rad * cos(w_n_b * t_init) + theta_dot_i * sin(w_n_b * t_init) / w_n_b) * rad2deg);
        }
    }
    public float f(float time, float theta, float theta_dot)
    {
        return theta_dot;
    }
    public float g(float time, float theta, float theta_dot)
    {
        return -k_b * theta / J_t;
        //return theta_i * cos(w_n_b * t) + theta_dot_i * sin(w_n_b * t) / w_n_b;
    }

    #region Math Functions
    public float rad2deg => Mathf.Rad2Deg;
    public float deg2rad => Mathf.Deg2Rad;
    public float cos(float x)
    {
        return Mathf.Cos(x);
    }
    public float sin(float x)
    {
        return Mathf.Sin(x);
    }
    public float tan(float x)
    {
        return Mathf.Tan(x);
    }
    public float atan(float x)
    {
        return Mathf.Atan(x);
    }
    public float ln(float x)
    {
        return Mathf.Log(x);
    }
    public float sqrt(float x)
    {
        return Mathf.Sqrt(x);
    }
    public float pow(float x, float y)
    {
        return Mathf.Pow(x, y);
    }
    public float log(float x, float y)
    {
        return Mathf.Log(x, y);
    }
    #endregion
}
