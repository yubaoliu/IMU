using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEditor;
using UnityEngine;

public class cameraScene : MonoBehaviour {
  Thread refresh;
	// Use this for initialization
	void Start () {
    refresh = new Thread(new ThreadStart(refreshAssetData));
    refresh.IsBackground = true;
    refresh.Start();
  }
	
	// Update is called once per frame
	void Update () {
    Texture2D img = Resources.Load("scene") as Texture2D;
    transform.GetComponent<Renderer>().material.mainTexture = img;
   
  }
  private void refreshAssetData()
  {
    while(true)
    {
      AssetDatabase.Refresh();
    }
  }

  void OnDestroy()
  {
    refresh.Join();
    refresh.Abort();
    
  }

}
