using UnityEngine;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;

public static class SaveLoadSystem
{
    public static void SaveVariables(SavingVariables SV)
    {
        BinaryFormatter formatter = new BinaryFormatter();
        string path = Application.persistentDataPath + "/" + SV.FileName;
        FileStream stream = new FileStream(path, FileMode.Create);

        formatter.Serialize(stream, SV);
        stream.Close();
    }

    public static SavingVariables LoadVariables(string fileName)
    {
        string path = Application.persistentDataPath + "/" + fileName;
        if (File.Exists(path))
        {
            BinaryFormatter formatter = new BinaryFormatter();
            FileStream stream = new FileStream(path, FileMode.Open);

            SavingVariables sv = formatter.Deserialize(stream) as SavingVariables;

            stream.Close();

            return sv;
        }
        else
        {
            Debug.Log("error no file");
            return null;
        }
    }
}
