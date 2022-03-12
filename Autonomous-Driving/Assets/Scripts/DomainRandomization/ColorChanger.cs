using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ColorChanger : MonoBehaviour
{
    Renderer[] renderers;
    GameObject[] environments;

    public TerrainLayer[] terrainLayers;
    
    public TerrainData terrainData;
     
 
    // Start is called before the first frame update
    void Start()
    {
        
        InvokeRepeating("ChangeTerrain", 0f, 1f);
        InvokeRepeating("ChangeColor", 0f, 2f);
        environments = GameObject.FindGameObjectsWithTag("Environment");
        terrainData = Terrain.activeTerrain.terrainData;
    }

    void ChangeColor()
    {
        foreach (GameObject environment in environments)
        {
            renderers = environment.GetComponentsInChildren<Renderer>();

            foreach (Renderer renderer in renderers)
            {
                renderer.material.SetColor("_BaseColor", Random.ColorHSV());
            }
        }
    }
    void ChangeTerrain()
    {
        TerrainLayer[] tempTerrainLayers = {null};
        int randomTerrain = Random.Range(0,terrainLayers.Length);
        tempTerrainLayers[0] = terrainLayers[randomTerrain];
        terrainData.terrainLayers = tempTerrainLayers;
    }

}
