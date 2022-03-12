using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
 
public class CameraCapture : MonoBehaviour
{
    public int fileCounter;
    private Camera camera;
    void Start()
    {
        InvokeRepeating("Capture", 0f, 1f);
        camera =  this.gameObject.GetComponent<Camera>();
    }
 
    public void Capture()
    {
        RenderTexture activeRenderTexture = RenderTexture.active;
        RenderTexture.active = camera.targetTexture;
 
        camera.Render();
 
        Texture2D image = new Texture2D(camera.targetTexture.width, camera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, camera.targetTexture.width, camera.targetTexture.height), 0, 0);
        image.Apply();
        RenderTexture.active = activeRenderTexture;
 
        byte[] bytes = image.EncodeToPNG();
        Destroy(image);
 
        File.WriteAllBytes(Application.dataPath + "/CameraImages/Donkey_11_Inner" + fileCounter + ".png", bytes);
        fileCounter++;
    }
}