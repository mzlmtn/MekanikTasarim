using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;

public class GraphDrawer : MonoBehaviour
{
    public GameObject ParentObject, xAxisObject, yAxisObject;
    public Text x1Text, x2Text, x3Text, x4Text, y1Text, y2Text, y3Text, y4Text;
    public LineRenderer Curve;
    public List<decimal> xCoords = new List<decimal>(), yCoords = new List<decimal>();

    public GraphDrawer(List<decimal> xs, List<decimal> ys)
    {
        xCoords = xs;
        yCoords = ys;
    }

    public void DrawGraph()
    {
        Vector3 position = ParentObject.transform.position;
        Vector2 size = new Vector2(xAxisObject.transform.localScale.x, yAxisObject.transform.localScale.y);

        int countX = xCoords.Count, countY = yCoords.Count;
        decimal yMin = yCoords.Min(), yMax = yCoords.Max(), xMin = xCoords.Min(), xMax = xCoords.Max();
        /*x1Text.text = xCoords[(int)(countX / 4)].ToString("F1");
        x2Text.text = xCoords[(int)(countX / 2)].ToString("F1");
        x3Text.text = xCoords[(int)(countX * 3 / 4)].ToString("F1");
        x4Text.text = xCoords[(int)(countX - 1)].ToString("F1");
        y1Text.text = yMin.ToString("F1");
        y2Text.text = (yMin / 2).ToString("F1");
        y3Text.text = (yMax / 2).ToString("F1");
        y4Text.text = yMax.ToString("F1");*/

        float x_space = size.x / xCoords.Count;
        float y_mult = size.y / (float)(DecimalMath.Abs(yMax) > DecimalMath.Abs(yMin) ?
            DecimalMath.Abs(yMax) * 2 : DecimalMath.Abs(yMin) * 2);

        Curve.positionCount = xCoords.Count;

        Curve.SetPosition(0, new Vector3(position.x, position.y + (float)yCoords[0] * y_mult, 0));

        for (int i = 1; i < countX; i++)
        {
            Curve.SetPosition(i, new Vector3(Curve.GetPosition(i - 1).x + x_space, position.y + (float)yCoords[i] * y_mult, 0));
        }
    }
}
