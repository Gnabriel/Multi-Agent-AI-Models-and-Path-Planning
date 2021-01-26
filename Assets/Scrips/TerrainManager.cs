using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json; // Import JSON.NET from Unity Asset store


public class TerrainManager : MonoBehaviour {


    //public TestScriptNoObject testNoObject = new TestScriptNoObject();

    public string terrain_filename = "Text/terrain";
    public TerrainInfo myInfo;

    public GameObject flag;


    // Use this for initialization
    void Start()
    {

    }

    // Use this for initialization
    void Awake()
    {

        var jsonTextFile = Resources.Load<TextAsset>(terrain_filename);

        myInfo = TerrainInfo.CreateFromJSON(jsonTextFile.text);

        myInfo.CreateCubes();


        // this code is used to create new terrains and obstacles
        //myInfo.TerrainInfo2();
        //myInfo.file_name = "test88";
        //string myString = myInfo.SaveToString();
        //myInfo.WriteDataToFile(myString);

        Instantiate(flag, myInfo.start_pos, Quaternion.identity);
        Instantiate(flag, myInfo.goal_pos, Quaternion.identity);



    }



    // Update is called once per frame
    void Update () {
		
	}
}



[System.Serializable]
public class TerrainInfo
{
    public string file_name;
    public float x_low;
    public float x_high;
    public int x_N;
    public float z_low;
    public float z_high;
    public int z_N;
    public float[,] traversability;

    public Vector3 start_pos;
    public Vector3 goal_pos;


    //public TerrainInfo()
    //{
    //    return;
    //}

    public void TerrainInfo2()
    {
        file_name = "terrain.json";
        x_low = 50f;
        x_high = 250f;
        x_N = 45;
        z_low = 50f;
        z_high = 250f;
        z_N = 7;
        Debug.Log("Using hard coded info...");
        //traversability = new float[,] { { 1.1f, 2f }, { 3.3f, 4.4f } };
        traversability = new float[x_N, z_N]; // hardcoded now, needs to change
        for(int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if ((i == 0 || i == x_N -1) || (j == 0 || j == z_N - 1))
                {
                    traversability[i, j] = 1.0f;
                }
                else
                {
                    traversability[i, j] = 0.0f;
                }
            }
        }
    }

    public int get_i_index(float x)
    {
        int index = (int) Mathf.Floor(x_N * (x - x_low) / (x_high - x_low));
        if (index < 0)
        {
            index = 0;
        }else if (index > x_N - 1)
        {
            index = x_N - 1;
        }
        return index;

    }
    public int get_j_index(float z) // get index of given coordinate
    {
        int index = (int)Mathf.Floor(z_N * (z - z_low) / (z_high - z_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > z_N - 1)
        {
            index = z_N - 1;
        }
        return index;
    }

    public float get_x_pos(int i)
    {
        float step = (x_high - x_low) / x_N;
        return x_low + step / 2 + step * i;
    }

    public float get_z_pos(int j) // get position of given index
    {
        float step = (z_high - z_low) / z_N;
        return z_low + step / 2 + step * j;
    }

    public void CreateCubes()
    {
        float x_step = (x_high - x_low) / x_N;
        float z_step = (z_high - z_low) / z_N;
        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if (traversability[i, j] > 0.5f)
                {
                    GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cube.transform.position = new Vector3(get_x_pos(i), 0.0f, get_z_pos(j));
                    cube.transform.localScale = new Vector3(x_step, 15.0f, z_step);
                }

            }
        }
    }



    public static TerrainInfo CreateFromJSON(string jsonString)
    {
        //Debug.Log("Reading json");
        return JsonConvert.DeserializeObject<TerrainInfo>(jsonString);
    }

    public string SaveToString()
    {
        //return JsonUtility.ToJson(this);
        return JsonConvert.SerializeObject(this);
    }

    public void WriteDataToFile(string jsonString)
    {
        string path = Application.dataPath + "/Resources/Text/saved_terrain.json";
        Debug.Log("AssetPath:" + path);
        System.IO.File.WriteAllText(path, jsonString);
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }


}