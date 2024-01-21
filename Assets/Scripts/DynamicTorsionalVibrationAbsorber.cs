using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DynamicTorsionalVibrationAbsorber : MonoBehaviour
{
    public decimal theta_Disc_i, theta_dot_Disc_i, theta_Fan_i, theta_dot_Fan_i, I_ş, 
        J_fan, J_disc, k_1, k_2, Gş, Rho, ExcFreq, ExcTorq;


    public bool Solve, StartSimulating, ShowGraph;
    public decimal deltaT;
    public int iterationCount;

    public Text TXTDiscDegree, TXTFanDegree;

    public InputField INPTIterationCount, INPTDeltaT, INPTThetaInitialDisc, INPTThetaInitialFan, 
        INPTThetaDotInitialDisc, INPTThetaDotInitialFan, INPTGShaft, INPTRho, INPTExcitationFreq, 
        INPTExcitationTorque;

    public GameObject Disc, Fan;

    public decimal dShaft1 => (decimal)GameObject.Find("Shaft1").transform.localScale.x;
    public decimal dShaft2 => (decimal)GameObject.Find("Shaft2").transform.localScale.x;
    public decimal LShaft1 => (decimal)GameObject.Find("Shaft1").transform.localScale.y * 2;
    public decimal LShaft2 => (decimal)GameObject.Find("Shaft2").transform.localScale.y * 2;
    public decimal LDisc => (decimal)GameObject.Find("Disc").transform.localScale.y * 2;
    // dDisc=?
    public decimal dFan => (decimal)GameObject.Find("Fan").transform.localScale.x;
    public decimal LFan => (decimal)GameObject.Find("Fan").transform.localScale.y * 2;

    public decimal VFan => pi * dFan * dFan * LFan / 4;




    void Start()
    {
        Disc = GameObject.Find("Disc");
        Fan = GameObject.Find("Fan");
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
        if (StartSimulating && i_counter < _thetaFan.Count - 1)
        {
            i_counter++;
            Disc.transform.eulerAngles = new Vector3((float)_thetaDisc[i_counter] * (float)r2d, Disc.transform.eulerAngles.y,
                Disc.transform.eulerAngles.z);
            Fan.transform.eulerAngles = new Vector3((float)_thetaFan[i_counter] * (float)r2d, Fan.transform.eulerAngles.y,
                Fan.transform.eulerAngles.z);

            TXTDiscDegree.text = "Yutucu Açısı: " + (_thetaDisc[i_counter] * r2d).ToString("F4");
            TXTFanDegree.text = "Pervane Açısı: " + (_thetaFan[i_counter] * r2d).ToString("F4");
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
        theta_Disc_i = decimal.Parse(INPTThetaInitialDisc.text.Replace('.', ','));
        theta_Fan_i = decimal.Parse(INPTThetaInitialFan.text.Replace('.', ','));
        theta_dot_Disc_i = decimal.Parse(INPTThetaDotInitialDisc.text.Replace('.', ','));
        theta_dot_Fan_i = decimal.Parse(INPTThetaDotInitialFan.text.Replace('.', ','));
        Gş = decimal.Parse(INPTGShaft.text.Replace('.', ',')) * 1000000000;
        Rho = decimal.Parse(INPTRho.text.Replace('.', ','));
        ExcFreq = decimal.Parse(INPTExcitationFreq.text.Replace('.', ','));
        ExcTorq = decimal.Parse(INPTExcitationTorque.text.Replace('.', ','));


        _time.Clear();
        _thetaFan.Clear();
        _thetaDisc.Clear();

        if (Gş > 0 && Rho > 0 && ExcFreq > 0 && ExcTorq > 0)
        {
            I_ş = pi * pow(dShaft1, 4) / 32;
            Debug.Log("Iş:" + I_ş + " Gş:" + Gş);
            k_1 = Gş * I_ş / LShaft1;
            k_2 = Gş * I_ş / LShaft2;
            Debug.Log("k1:" + k_1 + " k2:" + k_2);
            J_disc = 238.732416m;
            J_disc = Gş * I_ş * LShaft1 * 2 / ExcFreq / ExcFreq / (LShaft1 * (LShaft1 + LShaft2) - LShaft1 * LShaft1);
            Debug.Log("J_disc:" + J_disc);
            J_fan = VFan * Rho * dFan * dFan / 8;

            decimal wn2 = sqrt(k_2 / J_fan);
            decimal wn1 = sqrt(k_1 / J_disc);


            Debug.Log("J_fan:" + J_fan + " wn2:" + wn2 + " wn1:" + wn1);
            StartSimulating = false;

            RK4Solver(0, theta_Fan_i * d2r, theta_dot_Fan_i, theta_Disc_i * d2r, theta_dot_Disc_i, deltaT,
                iterationCount);

            GameObject.Find("Graph1").GetComponent<GraphDrawer>().xCoords = _time;
            GameObject.Find("Graph1").GetComponent<GraphDrawer>().yCoords = _thetaFan;
            GameObject.Find("Graph1").GetComponent<GraphDrawer>().DrawGraph();

            GameObject.Find("Graph2").GetComponent<GraphDrawer>().xCoords = _time;
            GameObject.Find("Graph2").GetComponent<GraphDrawer>().yCoords = _thetaDisc;
            GameObject.Find("Graph2").GetComponent<GraphDrawer>().DrawGraph();
        }
    }

    
    // 

    public List<decimal> _time = new List<decimal>(), _thetaDisc = new List<decimal>(), _thetaFan = new List<decimal>();
    public void RK4Solver(decimal t_init, decimal fan_theta_init, decimal fan_theta_dot_init,
        decimal disc_theta_init, decimal disc_theta_dot_init, decimal h, decimal iterationCount)
    {
        for (int i = 0; i < iterationCount; i++)
        {
            decimal k1 = h * f1(t_init, fan_theta_init, fan_theta_dot_init, disc_theta_init, disc_theta_dot_init);
            decimal l1 = h * g1(t_init, fan_theta_init, fan_theta_dot_init, disc_theta_init, disc_theta_dot_init);
            decimal m1 = h * h1(t_init, fan_theta_init, fan_theta_dot_init, disc_theta_init, disc_theta_dot_init);
            decimal n1 = h * j1(t_init, fan_theta_init, fan_theta_dot_init, disc_theta_init, disc_theta_dot_init);

            decimal k2 = h * f1(t_init + h / 2, fan_theta_init + k1 / 2, fan_theta_dot_init + l1 / 2, disc_theta_init + m1 / 2, disc_theta_dot_init + n1 / 2);
            decimal l2 = h * g1(t_init + h / 2, fan_theta_init + k1 / 2, fan_theta_dot_init + l1 / 2, disc_theta_init + m1 / 2, disc_theta_dot_init + n1 / 2);
            decimal m2 = h * h1(t_init + h / 2, fan_theta_init + k1 / 2, fan_theta_dot_init + l1 / 2, disc_theta_init + m1 / 2, disc_theta_dot_init + n1 / 2);
            decimal n2 = h * j1(t_init + h / 2, fan_theta_init + k1 / 2, fan_theta_dot_init + l1 / 2, disc_theta_init + m1 / 2, disc_theta_dot_init + n1 / 2);

            decimal k3 = h * f1(t_init + h / 2, fan_theta_init + k2 / 2, fan_theta_dot_init + l2 / 2, disc_theta_init + m2 / 2, disc_theta_dot_init + n2 / 2);
            decimal l3 = h * g1(t_init + h / 2, fan_theta_init + k2 / 2, fan_theta_dot_init + l2 / 2, disc_theta_init + m2 / 2, disc_theta_dot_init + n2 / 2);
            decimal m3 = h * h1(t_init + h / 2, fan_theta_init + k2 / 2, fan_theta_dot_init + l2 / 2, disc_theta_init + m2 / 2, disc_theta_dot_init + n2 / 2);
            decimal n3 = h * j1(t_init + h / 2, fan_theta_init + k2 / 2, fan_theta_dot_init + l2 / 2, disc_theta_init + m2 / 2, disc_theta_dot_init + n2 / 2);

            decimal k4 = h * f1(t_init + h, fan_theta_init + k3, fan_theta_dot_init + l3, disc_theta_init + m3, disc_theta_dot_init + n3);
            decimal l4 = h * g1(t_init + h, fan_theta_init + k3, fan_theta_dot_init + l3, disc_theta_init + m3, disc_theta_dot_init + n3);
            decimal m4 = h * h1(t_init + h, fan_theta_init + k3, fan_theta_dot_init + l3, disc_theta_init + m3, disc_theta_dot_init + n3);
            decimal n4 = h * j1(t_init + h, fan_theta_init + k3, fan_theta_dot_init + l3, disc_theta_init + m3, disc_theta_dot_init + n3);

            fan_theta_init += (k1 + 2 * k2 + 2 * k3 + k4) / 6;
            fan_theta_dot_init += (l1 + 2 * l2 + 2 * l3 + l4) / 6;
            disc_theta_init += (m1 + 2 * m2 + 2 * m3 + m4) / 6;
            disc_theta_dot_init += (n1 + 2 * n2 + 2 * n3 + n4) / 6;

            //Debug.Log("k1:" + k1 + " k2:" + k2 + " k3:" + k3 + " k4:" + k4);

            t_init += h;

            _time.Add(t_init);
            _thetaDisc.Add(disc_theta_init );
            _thetaFan.Add(fan_theta_init);
            //Debug.Log(_time[_time.Count - 1] + " | DİSC:" + disc_theta_init + " FAN:" + fan_theta_init);
        }
    }

    public decimal f1(decimal t, decimal fan_theta, decimal fan_theta_dot, decimal disc_theta, 
        decimal disc_theta_dot)
    {
        return fan_theta_dot;
    }
    public decimal g1(decimal t, decimal fan_theta, decimal fan_theta_dot, decimal disc_theta,
        decimal disc_theta_dot)
    {
        Debug.Log(Gş + " | " + I_ş + " | " + k_1 + " | " + k_2 + " | " + fan_theta + " | " + disc_theta + " | " + J_fan);
        return (ExcTorq * sin(ExcFreq * t) - k_2 * (fan_theta - disc_theta)) / J_fan;
    }
    public decimal h1(decimal t, decimal fan_theta, decimal fan_theta_dot, decimal disc_theta,
        decimal disc_theta_dot)
    {
        return disc_theta_dot;
    }
    public decimal j1(decimal t, decimal fan_theta, decimal fan_theta_dot, decimal disc_theta,
        decimal disc_theta_dot)
    {
        return (-k_2 * (disc_theta - fan_theta) - k_1 * disc_theta) / J_disc;
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
    public decimal r2d => 180 / DecimalMath.PI;
    public decimal d2r => DecimalMath.PI / 180;
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
