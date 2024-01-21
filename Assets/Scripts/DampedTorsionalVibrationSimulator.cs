using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DampedTorsionalVibrationSimulator : MonoBehaviour
{
    public decimal theta_i, theta_dot_i, I_ş, J_p, J_ş, J_t, k_b, c_b, Gş, Rho;

    public bool Solve, StartSimulating, ShowGraph;
    public decimal deltaT;
    public int iterationCount;

    public Text TXTAnalyticalDegree, TXTNumericalDegree;
    public InputField INPTIterationCount, INPTDeltaT, INPTThetaInitial, INPTThetaDotInitial, INPTGShaft, INPTCShaft;

    public GameObject ShaftN, MassN, ShaftA, MassA;
    public decimal dş => (decimal)GameObject.Find("Shaft").transform.localScale.x;
    public decimal Lş => (decimal)GameObject.Find("Shaft").transform.localScale.y * 2;
    public decimal dp => (decimal)GameObject.Find("Mass").transform.localScale.x;
    public decimal Lp => (decimal)GameObject.Find("Mass").transform.localScale.y * 2;
    public decimal Vş => pi * dş * dş * Lş / 4;
    public decimal Vp => pi * dp * dp * Lp / 4;

    void Start()
    {
        deltaT = 0.001m;
        Rho = 7850;
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
            Application.targetFrameRate = /*(int)DecimalMath.Round(FPScount)*/33;
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
            MassN.transform.eulerAngles = new Vector3((float)_theta[i_counter], MassN.transform.eulerAngles.y,
                MassN.transform.eulerAngles.z);
            MassA.transform.eulerAngles = new Vector3((float)_thetaA[i_counter], MassA.transform.eulerAngles.y,
                MassA.transform.eulerAngles.z);

            TXTNumericalDegree.text = "Açı: " + _theta[i_counter].ToString("F4");
            TXTAnalyticalDegree.text = "Açı: " + _thetaA[i_counter].ToString("F4");
        }
        else
        {
            StartSimulating = false;
        }
    }

    public void Calculate()
    {
        iterationCount = int.Parse(INPTIterationCount.text.Replace('.', ','));
        deltaT = decimal.Parse(INPTDeltaT.text.Replace('.', ','));
        theta_i = decimal.Parse(INPTThetaInitial.text.Replace('.', ','));
        theta_dot_i = decimal.Parse(INPTThetaDotInitial.text.Replace('.', ','));
        Gş = decimal.Parse(INPTGShaft.text.Replace('.', ',')) * 1000000000;
        c_b = decimal.Parse(INPTCShaft.text.Replace('.', ','));

        _time.Clear();
        _theta.Clear();
        _thetaA.Clear();

        if (Gş != 0 && (theta_i != 0 || theta_dot_i != 0))
        {
            I_ş = pi * pow(dş, 4) / 32;
            k_b = Gş * I_ş / Lş;
            J_p = Vp * Rho * dp * dp / 8;
            J_ş = Vş * Rho * dş * dş / 8;
            J_t = J_p + J_ş;
            //w_n_b = sqrt(k_b / J_t);

            StartSimulating = false;

            RungeKuttaSolverSecondDegreeODE(0, theta_i * deg2rad, theta_dot_i, deltaT, iterationCount);

            GameObject.Find("Graph1").GetComponent<GraphDrawer>().xCoords = _time;
            GameObject.Find("Graph1").GetComponent<GraphDrawer>().yCoords = _thetaA;
            GameObject.Find("Graph1").GetComponent<GraphDrawer>().DrawGraph();

            GameObject.Find("Graph2").GetComponent<GraphDrawer>().xCoords = _time;
            GameObject.Find("Graph2").GetComponent<GraphDrawer>().yCoords = _theta;
            GameObject.Find("Graph2").GetComponent<GraphDrawer>().DrawGraph();

        }
        //Debug.Log($"{I_ş} _ {k_b} _ {deltaT} _ {iterationCount}");

    }

    /*  ω_(n, b) = sqrt(k_b / j_b)  [rad/s]
        k_b = G * I_ş / L  [N*m/rad]
        I_ş = π * d^4 / 32  [m^4]
        J = m * r^2 / 2
    */

    public List<decimal> _time = new List<decimal>(), _theta = new List<decimal>(), _thetaA = new List<decimal>();

    public void RungeKuttaSolverSecondDegreeODE(decimal t_init, decimal theta_init, decimal theta_dot_init,
        decimal h, decimal iterationCount)
    {
        for (int i = 0; i < iterationCount; i++)
        {
            decimal k1 = h * f(t_init, theta_init, theta_dot_init);
            decimal l1 = h * g(t_init, theta_init, theta_dot_init);

            decimal k2 = h * f(t_init + h / 2, theta_init + k1 / 2, theta_dot_init + l1 / 2);
            decimal l2 = h * g(t_init + h / 2, theta_init + k1 / 2, theta_dot_init + l1 / 2);

            decimal k3 = h * f(t_init + h / 2, theta_init + k2 / 2, theta_dot_init + l2 / 2);
            decimal l3 = h * g(t_init + h / 2, theta_init + k2 / 2, theta_dot_init + l2 / 2);

            decimal k4 = h * f(t_init + h, theta_init + k3, theta_dot_init + l3);
            decimal l4 = h * g(t_init + h, theta_init + k3, theta_dot_init + l3);

            theta_init += (k1 + 2 * k2 + 2 * k3 + k4) / 6;
            theta_dot_init += (l1 + 2 * l2 + 2 * l3 + l4) / 6;

            t_init += h;

            _time.Add(t_init);
            _theta.Add(theta_init * rad2deg);

            decimal left = c_b * c_b, right = 4 * J_t * k_b;
            decimal res = 0;

            if (left > right) //aşırı sönümlü sistem
            {
                decimal a = c_b / 2 / J_t;
                decimal b = sqrt(abs(c_b * c_b - 4 * J_t * k_b)) / 2 / J_t;
                decimal r1 = -a + b, r2 = -a - b;
                decimal c1 = theta_i * deg2rad + (theta_i * deg2rad * r1 - theta_dot_i) / (r2 - r1);
                decimal c2 = (theta_dot_i - theta_i * deg2rad * r1) / (r2 - r1);
                decimal e = exp(-a * t_init);
                decimal _left = c1 * exp(b * t_init), _right = c2 * exp(-b * t_init);
                res = e * (_left + _right);
            }
            if (left == right) //kritik sönümlü sistem
            {
                decimal a = c_b / 2 / J_t;
                decimal b = sqrt(abs(c_b * c_b - 4 * J_t * k_b)) / 2 / J_t;
                decimal r1 = -a + b, r2 = -a - b;
                decimal e = exp(-a * t_init);
                decimal c1 = theta_i * deg2rad;
                decimal c2 = theta_dot_i + c_b * theta_i * deg2rad / 2 / J_t;
                decimal _left = c1, _right = c2 * t_init;
                res = e * (_left + _right);
            }
            if (left < right) // az sönümlü sistem
            {
                decimal a = c_b / 2 / J_t;
                decimal b = sqrt(abs(c_b * c_b - 4 * J_t * k_b)) / 2 / J_t;
                decimal r1 = -a + b, r2 = -a - b;
                decimal e = exp(-a * t_init);
                decimal c1 = theta_i * deg2rad;
                decimal c2 = (theta_dot_i + theta_i * deg2rad * c_b / 2 / J_t) / b;
                decimal _left = c1 * cos(b * t_init), _right = c2 * sin(b * t_init);
                res = e * (_left + _right);
            }
            _thetaA.Add(res * rad2deg);
        }
    }
    public decimal f(decimal time, decimal theta, decimal theta_dot)
    {
        return theta_dot;
    }
    public decimal g(decimal time, decimal theta, decimal theta_dot)
    {
        decimal a = -k_b * theta / J_t;
        decimal b = -c_b * theta_dot / J_t;
        return a + b;
        //return theta_i * cos(w_n_b * t) + theta_dot_i * sin(w_n_b * t) / w_n_b;
    }

    public void BTNSolve()
    {
        Solve = true;
    }
    public void BTNSimulate()
    {
        StartSimulating = true;
    }

    #region Math Functions
    public decimal pi => DecimalMath.PI;
    public decimal rad2deg => 180 / DecimalMath.PI;
    public decimal deg2rad => DecimalMath.PI / 180;
    public decimal abs(decimal x)
    {
        return DecimalMath.Abs(x);
    }
    public decimal cos(decimal x)
    {
        return DecimalMath.Cos(x);
    }
    public decimal sin(decimal x)
    {
        return DecimalMath.Sin(x);
    }
    public decimal tan(decimal x)
    {
        return DecimalMath.Tan(x);
    }
    public decimal atan(decimal x)
    {
        return DecimalMath.Atan(x);
    }
    public decimal ln(decimal x)
    {
        return DecimalMath.Log(x);
    }
    public decimal exp(decimal x)
    {
        return (decimal)DecimalMath.Exp(x);
    }
    public decimal sqrt(decimal x)
    {
        return DecimalMath.Sqrt(x);
    }
    public decimal pow(decimal x, decimal y)
    {
        return DecimalMath.Pow(x, y);
    }
    #endregion
}
