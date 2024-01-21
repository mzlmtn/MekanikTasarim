using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Simulator : MonoBehaviour
{
    public bool Solve, isSimulationStarted;
    public int IterationCount, SimulationCounter;
    public float PassedTime;
    public float h, M, m3, m4, k1, k2, k3, k4, F0, w, X1_Init, X2_Init, X3_Init, X4_Init, 
        X1_Dot_Init, X2_Dot_Init, X3_Dot_Init, X4_Dot_Init;

    public List<float> T1, X1, X1_D, X2, X2_D, X3, X3_D, X4, X4_D;
    //public List<GameObject>

    public Transform Shaft, Mass3, Mass4, FigureTransform, SimulationObjectsTransform, 
        CanvasEnterVariablesTransform, CanvasSimulationTransform;
    public GameObject BTNStartStop, BTNStartSim;

    public InputField IM, IM3, IM4, Ik1, Ik2, Ik3, Ik4, IF0, Iw, IX1_Init, IX2_Init, IX3_Init, IX4_Init,
        IX1_Dot_Init, IX2_Dot_Init, IX3_Dot_Init, IX4_Dot_Init, Ih, IIterationCount, IFileName;
    public Text TXTTimeScale, TXTFPS, TXTIterationCount;

    private float firstTimeScale, FPScount;


    public void BTNSolve()
    {
        Solve = true;
        BTNStartSim.GetComponent<Button>().interactable = true;
    }

    public void BTNStart()
    {
        FigureTransform.gameObject.SetActive(false);
        SimulationObjectsTransform.gameObject.SetActive(true);
        CanvasEnterVariablesTransform.gameObject.SetActive(false);
        CanvasSimulationTransform.gameObject.SetActive(true);

        firstTimeScale = 0.03f / Time.fixedDeltaTime;
        Time.timeScale = firstTimeScale;
        TXTTimeScale.text = Time.timeScale.ToString("F1");

        StartCoroutine(OnStarted());

        isSimulationStarted = true;
    }

    private IEnumerator OnStarted()
    {
        while (true)
        {
            FPScount = 1f / Time.unscaledDeltaTime;
            TXTFPS.text = "Frames Per Second: 33"/* + Mathf.Round(FPScount)*/;
            Application.targetFrameRate = /*(int)Mathf.Round(FPScount)*/33;
            QualitySettings.vSyncCount = 0;
            yield return new WaitForSeconds(0.25f);
        }
    }

    private void FixedUpdate()
    {
        if (isSimulationStarted)
        {
            PassedTime += Time.deltaTime;

            TXTIterationCount.text = "Iteration Count: " + SimulationCounter;

            if (T1.Count > 0 && SimulationCounter < T1.Count - 1)
            {
                if (Mathf.Abs(T1[SimulationCounter] - PassedTime) <= 0.001f)
                {
                    SimulationCounter++;

                    Shaft.position = new Vector3(X2[SimulationCounter], X1[SimulationCounter], 0);
                    Mass3.position = new Vector3(X3[SimulationCounter] + 4, 0, 0);
                    Mass4.position = new Vector3(0, X4[SimulationCounter] - 4, 0);
                }
            }
        }

        if (Solve)
        {
            Solve = false;
            LoadVariablesFromInputs();
            SolveEquation();
        }
    }


    //  X1:
    //  ((k4-m4)(k2k3-k2m3-k3m3-k3Mw^2+Mm3w^2))F0=0
    //  k3=(k2*m3*w^2-M*m3*w^4)/(k2-m3*w^2-M*w^2)
    //  X2:
    //  ((k3-m3)(k1k4-k1m4-k4m4-k4Mw^2+Mm4w^2))F0=0
    //  k4=(k1*m4*w^2-M*m4*w^4)/(k1-m4*w^2-M*w^2)


    public void SolveEquation()
    {
        T1.Clear();
        X1.Clear();
        X1_D.Clear();
        X2.Clear();
        X2_D.Clear();
        X3.Clear();
        X3_D.Clear();
        X4.Clear();
        X4_D.Clear();

        //k3 = 0.001f;
        //k4 = 0.001f;

        //k3 = (k2 * m3 * w * w - M * m3 * w * w * w * w) / (k2 - m3 * w * w - M * w * w);
        //k4 = (k1 * m4 * w * w - M * m4 * w * w * w * w) / (k1 - m4 * w * w - M * w * w);

        //k3 = k2 * m3 / M;
        //k4 = k1 * m4 / M;

        //k3 = 7.905f;
        //k4 = 7.905f;

        //w = Mathf.Sqrt(k1 / M);

        (k3, k4) = FindK3K4();

        Ik3.text = k3.ToString();
        Ik4.text = k4.ToString();
        Iw.text = w.ToString();

        RK4(0, X1_Init, X1_Dot_Init, X2_Init, X2_Dot_Init, X3_Init, X3_Dot_Init, X4_Init, X4_Dot_Init,
            h, IterationCount);
    }


    Dictionary<string, decimal> dict = new Dictionary<string, decimal>();
    public (float, float) FindK3K4()
    {

        List<string> Eqs = new List<string>();
        //[(k_4-m_4 w^2 )(k_2 k_3-k_2 m_3 w^2-k_3 m_3 w^2-k_3 Mw^2+Mm_3 w^4 )]=0
        //[(k_3 - m_3 w ^ 2)(k_1 k_4 - k_1 m_4 w^2 - k_4 m_4 w^2 - k_4 Mw ^ 2 + Mm_4 w ^ 4 )]= 0
        //k_4=x k_3=y
        string eq1 = "(x-" + (m4 * w * w) + ")*(" + k2 + "*y-" + (k2 * m3 * w * w) + "-y*" + 
            (m3 * w * w) + "-y*" + (M * w * w) + "+" + (M * m3 * w * w * w * w) + ")";
        string eq2 = "(y-" + (m3 * w * w) + ")*(" + k1 + "*x-" + (k1 * m4 * w * w) + "-x*" +
            (m4 * w * w) + "-x*" + (M * w * w) + "+" + (M * m4 * w * w * w * w) + ")";

        Debug.Log(eq1);
        Debug.Log(eq2);

        Eqs.Add(eq1);
        Eqs.Add(eq2);

        decimal iv1 = 2, iv2 = 2;

        dict.Add("x", iv1);
        dict.Add("y", iv2);

        int iterCount = 10;

        Dictionary<string, decimal> results = NumericalMethods.MethodMultiVar_SolveEquationSystem(Eqs, dict, iterCount);

        string sum = "";

        foreach (KeyValuePair<string, decimal> p in results)
        {
            sum += p.Key + ": " + p.Value + ", ";
        }

        Debug.Log(sum);
        return ((float)results["x"], (float)results["y"]);
    }

    //  M * x1_dd + k1 * x1 + k4 * (x1 - x4) = F0 * sin(w * t)
    //  M * x2_dd + k2 * x2 + k3 * (x2 - x3) = F0 * cos(w * t)
    //  m4 * x4_dd + k4 * (x4 - x1) = 0
    //  m3 * x3_dd + k3 * (x3 - x2) = 0

    //  f1 = x1_d
    //  f2 = (F0 * sin(w * t) - k1 * x1 - k4 * (x1 - x4)) / M
    //  f3 = x2_dd
    //  f4 = (F0 * cos(w * t) - k2 * x2 - k3 * (x2 - x3)) / M
    //  f5 = x4_dd
    //  f6 = -k4 * (x4 - x1) / m4
    //  f7 = x3_dd
    //  f8 = -k3 * (x3 - x2) / m3

    //  (k | x1_init), (l | x1_dot_init), (m | x2_init), (n | x2_dot_init),
    //  (p | x3_init), (r | x3_dot_init), (t | x4_init), (s | x4_dot_init)
    public void RK4(float t_init, float x1_init, float x1_dot_init, float x2_init, float x2_dot_init,
        float x3_init, float x3_dot_init, float x4_init, float x4_dot_init, float h, int iterationCount)
    {
        for (int i = 0; i < iterationCount; i++)
        {
            float _k1 = h * f1(x1_dot_init);
            float _l1 = h * f2(t_init, x1_init, x4_init);
            float _m1 = h * f3(x2_dot_init);
            float _n1 = h * f4(t_init, x2_init, x3_init);
            float _p1 = h * f5(x4_dot_init);
            float _r1 = h * f6(x4_init, x1_init);
            float _t1 = h * f7(x3_dot_init);
            float _s1 = h * f8(x3_init, x2_init);

            float _k2 = h * f1(x1_dot_init + _l1 / 2);
            float _l2 = h * f2(t_init + h / 2, x1_init + _k1 / 2, x4_init + _t1 / 2);
            float _m2 = h * f3(x2_dot_init + _n1 / 2);
            float _n2 = h * f4(t_init + h / 2, x2_init + _m1 / 2, x3_init + _p1 / 2);
            float _p2 = h * f5(x4_dot_init + _s1 / 2);
            float _r2 = h * f6(x4_init + _t1 / 2, x1_init + _k1 / 2);
            float _t2 = h * f7(x3_dot_init + _r1 / 2);
            float _s2 = h * f8(x3_init + _p1 / 2, x2_init + _m1 / 2);

            float _k3 = h * f1(x1_dot_init + _l2 / 2);
            float _l3 = h * f2(t_init + h / 2, x1_init + _k2 / 2, x4_init + _t2 / 2);
            float _m3 = h * f3(x2_dot_init + _n2 / 2);
            float _n3 = h * f4(t_init + h / 2, x2_init + _m2 / 2, x3_init + _p2 / 2);
            float _p3 = h * f5(x4_dot_init + _s2 / 2);
            float _r3 = h * f6(x4_init + _t2 / 2, x1_init + _k2 / 2);
            float _t3 = h * f7(x3_dot_init + _r2 / 2);
            float _s3 = h * f8(x3_init + _p2 / 2, x2_init + _m2 / 2);

            float _k4 = h * f1(x1_dot_init + _l3);
            float _l4 = h * f2(t_init + h, x1_init + _k3, x4_init + _t3);
            float _m4 = h * f3(x2_dot_init + _n3);
            float _n4 = h * f4(t_init + h, x2_init + _m3, x3_init + _p3);
            float _p4 = h * f5(x4_dot_init + _s3);
            float _r4 = h * f6(x4_init + _t3, x1_init + _k3);
            float _t4 = h * f7(x3_dot_init + _r3);
            float _s4 = h * f8(x3_init + _p3, x2_init + _m3);

            x1_init += (_k1 + 2 * _k2 + 2 * _k3 + _k4) / 6;
            x1_dot_init += (_l1 + 2 * _l2 + 2 * _l3 + _l4) / 6;
            x2_init += (_m1 + 2 * _m2 + 2 * _m3 + _m4) / 6;
            x2_dot_init += (_n1 + 2 * _n2 + 2 * _n3 + _n4) / 6;
            x3_init += (_p1 + 2 * _p2 + 2 * _p3 + _p4) / 6;
            x3_dot_init += (_r1 + 2 * _r2 + 2 * _r3 + _r4) / 6;
            x4_init += (_t1 + 2 * _t2 + 2 * _t3 + _t4) / 6;
            x4_dot_init += (_s1 + 2 * _s2 + 2 * _s3 + _s4) / 6;

            t_init += h;

            T1.Add(t_init);

            X1.Add(x1_init);
            X1_D.Add(x1_dot_init);
            X2.Add(x2_init);
            X2_D.Add(x2_dot_init);
            X3.Add(x3_init);
            X3_D.Add(x3_dot_init);
            X4.Add(x4_init);
            X4_D.Add(x4_dot_init);
        }
    }

    public float f1(float x1_d)
    {
        return x1_d;
    }
    public float f2(float t, float x1, float x4)
    {
        return (F0 * sin(w * t) - k1 * x1 - k4 * (x1 - x4)) / M;
    }
    public float f3(float x2_dd)
    {
        return x2_dd;
    }
    public float f4(float t, float x2, float x3)
    {
        return (F0 * cos(w * t) - k2 * x2 - k3 * (x2 - x3)) / M;
    }
    public float f5(float x4_dd)
    {
        return x4_dd;
    }
    public float f6(float x4, float x1)
    {
        return -k4 * (x4 - x1) / m4;
    }
    public float f7(float x3_dd)
    {
        return x3_dd;
    }
    public float f8(float x3, float x2)
    {
        return -k3 * (x3 - x2) / m3;
    }



    //► ▐▌ ▌▐  ▐▐  ≪  ≫
    public void StartStop()
    {
        if (BTNStartStop.GetComponentInChildren<Text>().fontSize == 30)
        {
            Time.timeScale = 0;
            BTNStartStop.GetComponentInChildren<Text>().fontSize = 50;
            BTNStartStop.GetComponentInChildren<Text>().text = "►";
        }
        else
        {
            Time.timeScale = firstTimeScale;
            BTNStartStop.GetComponentInChildren<Text>().fontSize = 30;
            BTNStartStop.GetComponentInChildren<Text>().text = "▌▐";
        }
        TXTTimeScale.text = (Time.timeScale / 30).ToString("F2");
    }
    public void SlowDown()
    {
        if (Time.timeScale > firstTimeScale / 6 && Time.timeScale != 0)
        {
            Time.timeScale -= firstTimeScale / 6;
        }
        TXTTimeScale.text = (Time.timeScale / 30).ToString("F2");
    }
    public void SpeedUp()
    {
        if (Time.timeScale < firstTimeScale && Time.timeScale != 0)
        {
            Time.timeScale += firstTimeScale / 6;
        }
        TXTTimeScale.text = (Time.timeScale / 30).ToString("F2");
    }


    public void LoadVariablesFromInputs()
    {
        SavingVariables sv = new SavingVariables("ab", float.Parse(Ih.text.Replace('.', ',')),
            float.Parse(IM.text.Replace('.', ',')), float.Parse(IM3.text.Replace('.', ',')),
            float.Parse(IM4.text.Replace('.', ',')), float.Parse(Ik1.text.Replace('.', ',')),
            float.Parse(Ik2.text.Replace('.', ',')), float.Parse(Ik3.text.Replace('.', ',')),
            float.Parse(Ik4.text.Replace('.', ',')), float.Parse(IF0.text.Replace('.', ',')),
            float.Parse(Iw.text.Replace('.', ',')), float.Parse(IX1_Init.text.Replace('.', ',')),
            float.Parse(IX2_Init.text.Replace('.', ',')), float.Parse(IX3_Init.text.Replace('.', ',')),
            float.Parse(IX4_Init.text.Replace('.', ',')), float.Parse(IX1_Dot_Init.text.Replace('.', ',')),
            float.Parse(IX2_Dot_Init.text.Replace('.', ',')), float.Parse(IX3_Dot_Init.text.Replace('.', ',')),
            float.Parse(IX4_Dot_Init.text.Replace('.', ',')));

        IterationCount = int.Parse(IIterationCount.text);
        h = sv.h;
        M = sv.M;
        m3 = sv.m3;
        m4 = sv.m4;
        k1 = sv.k1;
        k2 = sv.k2;
        k3 = sv.k3;
        k4 = sv.k4;
        F0 = sv.F0;
        w = sv.w;
        X1_Init = sv.X1_Init;
        X2_Init = sv.X2_Init;
        X3_Init = sv.X3_Init;
        X4_Init = sv.X4_Init;
        X1_Dot_Init = sv.X1_Dot_Init;
        X2_Dot_Init = sv.X2_Dot_Init;
        X3_Dot_Init = sv.X3_Dot_Init;
        X4_Dot_Init = sv.X4_Dot_Init;
    }


    public void SaveValues()
    {
        string fileName = "a";
        SavingVariables sv = new SavingVariables(fileName, float.Parse(Ih.text.Replace('.', ',')), 
            float.Parse(IM.text.Replace('.', ',')), float.Parse(IM3.text.Replace('.', ',')), 
            float.Parse(IM4.text.Replace('.', ',')), float.Parse(Ik1.text.Replace('.', ',')), 
            float.Parse(Ik2.text.Replace('.', ',')), float.Parse(Ik3.text.Replace('.', ',')), 
            float.Parse(Ik4.text.Replace('.', ',')), float.Parse(IF0.text.Replace('.', ',')), 
            float.Parse(Iw.text.Replace('.', ',')), float.Parse(IX1_Init.text.Replace('.', ',')), 
            float.Parse(IX2_Init.text.Replace('.', ',')), float.Parse(IX3_Init.text.Replace('.', ',')),
            float.Parse(IX4_Init.text.Replace('.', ',')), float.Parse(IX1_Dot_Init.text.Replace('.', ',')), 
            float.Parse(IX2_Dot_Init.text.Replace('.', ',')), float.Parse(IX3_Dot_Init.text.Replace('.', ',')), 
            float.Parse(IX4_Dot_Init.text.Replace('.', ',')));

        SaveLoadSystem.SaveVariables(sv);
    }

    public void LoadValues(string fileName)
    {
        SavingVariables sv = SaveLoadSystem.LoadVariables("a");

        Ih.text = sv.h.ToString();
        IM.text = sv.M.ToString();
        IM3.text = sv.m3.ToString();
        IM4.text = sv.m4.ToString();
        Ik1.text = sv.k1.ToString();
        Ik2.text = sv.k2.ToString();
        Ik3.text = sv.k3.ToString();
        Ik4.text = sv.k4.ToString();
        IF0.text = sv.F0.ToString();
        Iw.text = sv.w.ToString();
        IX1_Init.text = sv.X1_Init.ToString();
        IX2_Init.text = sv.X2_Init.ToString();
        IX3_Init.text = sv.X3_Init.ToString();
        IX4_Init.text = sv.X4_Init.ToString();
        IX1_Dot_Init.text = sv.X1_Dot_Init.ToString();
        IX2_Dot_Init.text = sv.X2_Dot_Init.ToString();
        IX3_Dot_Init.text = sv.X3_Dot_Init.ToString();
        IX4_Dot_Init.text = sv.X4_Dot_Init.ToString();
    }

    #region Matematiksel Kısaltmalar
    // dereceyi radyana çevirmek için çarpan (deg * PI / 180)
    public float deg2Rad => Mathf.Deg2Rad;
    // radyanı dereceye çevirmek için çarpan (rad * 180 / PI)
    public float rad2Deg => Mathf.Rad2Deg;
    // PI sayısı
    public float pi => Mathf.PI;
    // mutlak değer fonksiyonu
    public float abs(float x) => Mathf.Abs(x);
    // sinüs fonksiyonu
    public float sin(float x) => Mathf.Sin(x);
    // cosinüs fonksiyonu
    public float cos(float x) => Mathf.Cos(x);
    // tanjant fonksiyonu
    public float tan(float x) => Mathf.Tan(x);
    // arc-tanjant fonksiyonu
    public float atan(float x) => Mathf.Atan(x);
    // karekök fonksiyonu
    public float sqrt(float x) => Mathf.Sqrt(x);
    // üs alma fonksiyonu
    public float pow(float x, float y) => Mathf.Pow(x, y);
    #endregion
}

[System.Serializable]
public class SavingVariables
{
    public string FileName;
    public int IterationCount;
    public float h, M, m3, m4, k1, k2, k3, k4, F0, w, X1_Init, X2_Init, X3_Init, X4_Init,
        X1_Dot_Init, X2_Dot_Init, X3_Dot_Init, X4_Dot_Init;

    public SavingVariables(string _FileName, float _h, float _M, float _m3, float _m4, 
        float _k1, float _k2, float _k3, float _k4, float _F0, float _w, 
        float _X1_Init, float _X2_Init, float _X3_Init, float _X4_Init, 
        float _X1_Dot_Init, float _X2_Dot_Init, float _X3_Dot_Init, float _X4_Dot_Init)
    {
        FileName = _FileName;
        h = _h;
        M = _M;
        m3 = _m3;
        m4 = _m4;
        k1 = _k1;
        k2 = _k2;
        k3 = _k3;
        k4 = _k4;
        F0 = _F0;
        w = _w;
        X1_Init = _X1_Init;
        X2_Init = _X2_Init;
        X3_Init = _X3_Init;
        X4_Init = _X4_Init;
        X1_Dot_Init = _X1_Dot_Init;
        X2_Dot_Init = _X2_Dot_Init;
        X3_Dot_Init = _X3_Dot_Init;
        X4_Dot_Init = _X4_Dot_Init;
    }
}
