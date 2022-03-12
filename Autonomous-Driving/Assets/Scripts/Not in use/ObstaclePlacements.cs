using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObstaclePlacements : MonoBehaviour
{
    public GameObject obstacleSetup1;
    public GameObject obstacleSetup2;
    public GameObject obstacleSetup3;
    private int placementChoice;
    GameObject[] environments;
    GameObject[] obstacles;
    // Start is called before the first frame update
    void Start()
    {
        InvokeRepeating("ObstaclePlacement", 0f, 35f);
        environments = GameObject.FindGameObjectsWithTag("Environment");
    }

    // Update is called once per frame
    public void ObstaclePlacement()
    {
        DestroyObstacles();        
        foreach (GameObject environment in environments)
        {
            
            placementChoice = Random.Range(1,4);
            if (placementChoice == 1)
            {
                var obstaclesetup = Instantiate(obstacleSetup1, new Vector3(0f,0f,0f),Quaternion.identity);
                obstaclesetup.transform.parent = environment.transform;
            }
            if (placementChoice == 2)
            {
                var obstaclesetup = Instantiate(obstacleSetup2, new Vector3(0f,0f,0f),Quaternion.identity);
                obstaclesetup.transform.parent = environment.transform;
            }
            if (placementChoice == 3)
            {
                var obstaclesetup = Instantiate(obstacleSetup3, new Vector3(0f,0f,0f),Quaternion.identity);
                obstaclesetup.transform.parent = environment.transform;
            }
        }
        
    }
    public void DestroyObstacles()  
    {
        foreach (GameObject environment in environments)
        {
            obstacles = GameObject.FindGameObjectsWithTag("Obstacle");
            foreach (GameObject obstacle in obstacles)
            {
                Destroy(obstacle);
            }   
        }
    }
}
