using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LightController : MonoBehaviour
{
    public Light light;
    public GameObject blinkingLight;
    private Light lightComponent;
    public int  blinkIntensity = 1;
    [SerializeField] private float secondsInFullDay;
    [Range(0, 1)] public float currentTimeOfDay = 0f;
    private float lightInitialIntensity;
    public int lightDirection = 1;

    // Start is called before the first frame update
    void Start()
    {
        lightInitialIntensity = light.intensity;
        lightComponent = blinkingLight.GetComponent<Light>();
        InvokeRepeating("BlinkLight", 0f, 0.01f);
    }
    void BlinkLight()
    {
        blinkIntensity = Random.Range(0,5);
        lightComponent.intensity = blinkIntensity;
    }
    // Update is called once per frame
    void Update()
    {
        
        
        if (lightDirection == 1)
        {
            UpdateSun1();
        }
        else
        {
            UpdateSun2();
        }

        currentTimeOfDay += (Time.deltaTime / secondsInFullDay);
        if (currentTimeOfDay >= 1)
        {
            currentTimeOfDay = 0;
            if (lightDirection == 1)
            {
                lightDirection = 2;
            }
            else
            {
                lightDirection = 1;
            }    
        }
    }

    void UpdateSun1()
    {
        light.transform.localRotation = Quaternion.Euler((currentTimeOfDay * 360f) - 90, 170, 0);
        float intensityMultiplier = 1;
        if (currentTimeOfDay <= 0.13f || currentTimeOfDay >= 0.85f)
        {
            intensityMultiplier = 0;
        }
        else if (currentTimeOfDay <= 0.15f)
        {
            intensityMultiplier = Mathf.Clamp01((currentTimeOfDay - 0.13f) * (1 / 0.02f));
        }

        else if (currentTimeOfDay >= 0.83f)
        {
            intensityMultiplier = Mathf.Clamp01(1 - ((currentTimeOfDay - 0.83f) * (1 / 0.02f)));

        }
        light.intensity = lightInitialIntensity * intensityMultiplier;
    }
    void UpdateSun2()
    {
        light.transform.localRotation = Quaternion.Euler((currentTimeOfDay * 360f) - 90, 90, 90);
        float intensityMultiplier = 1;
        if (currentTimeOfDay <= 0.13f || currentTimeOfDay >= 0.85f)
        {
            intensityMultiplier = 0;
        }
        else if (currentTimeOfDay <= 0.15f)
        {
            intensityMultiplier = Mathf.Clamp01((currentTimeOfDay - 0.13f) * (1 / 0.02f));
        }

        else if (currentTimeOfDay >= 0.83f)
        {
            intensityMultiplier = Mathf.Clamp01(1 - ((currentTimeOfDay - 0.83f) * (1 / 0.02f)));

        }
        light.intensity = lightInitialIntensity * intensityMultiplier;
    }

}
