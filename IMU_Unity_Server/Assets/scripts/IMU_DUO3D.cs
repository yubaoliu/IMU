using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using DUODeviceLib;
using Assets.scripts;
using System.Runtime.InteropServices;

public class IMU_DUO3D : MonoBehaviour
{
  //float imuDeltaTime = 2.0f / 1000.0f;
  public struct IMU_SENSOR
  {
    public Vector3 linear_acceleration;
    public Vector3 angular_velocity;
    public Vector3 magn;

    public Vector3 _gyro_offset;
    public Vector3 _gyro_varience;
  }

  IMU_SENSOR imu_msg;

  Quaternion IMUrotation=Quaternion.identity;
  byte[] leftData;
  byte[] rightData;

  IntPtr left;
  IntPtr right;

  Texture2D liveVedioFromWebCamera;
  uint size;
  DUODevice device;

  static int frameCount = 0;


  void Start()
  {
    if (DllWrapper.IsMadgwickDllLibLoaded())
    {
      Debug.Log("MadgwickDllLib loaded successfully");
    }else
    {
      Debug.LogError("MadgwickDllLib loaded failed");
    }
    device = new DUODevice();

    //  liveVedioFromWebCamera = new Texture2D(640, 480,TextureFormat.Alpha8,false);
    liveVedioFromWebCamera = new Texture2D(640, 480);
    imu_msg._gyro_offset = new Vector3(-1.8966f, 1.9159f, 0.1865f);
    imu_msg._gyro_varience = new Vector3(0.0351f, 0.0149f, 0.0000001f);
  //initialize DUO3D camara

    device.DUODeviceStatusChanged += DUODeviceStatusChanged;
    device.DUOFrameReceived += DUOFrameReceived;
    device.Resolution = new DUOResolutionInfo() {
      width = 640,
      height = 480,
      //binning = DUOBinning.DUO_BIN_HORIZONTAL2 | DUOBinning.DUO_BIN_VERTICAL2,
      binning = DUOBinning.DUO_BIN_NONE,
      fps =30
    };
    
    device.Exposure = 80;
    device.Gain = 10;
    device.LED = 50;

    device.Start();
    imu_msg.magn = Vector3.zero;

    Debug.Log("inited");

  }


  Quaternion baseline = Quaternion.Inverse(Quaternion.identity);

  // Update is called once per frame
  void Update()
  {
    if (Input.GetKeyUp(KeyCode.Space))
    {
      baseline = Quaternion.Inverse(IMUrotation);
    }

    IMUrotation = DllWrapper.MadgwickFilter(Time.deltaTime, ref imu_msg.angular_velocity, ref imu_msg.linear_acceleration, ref imu_msg.magn, ref imu_msg._gyro_varience, IMUrotation.x, IMUrotation.y, IMUrotation.z, IMUrotation.w);
   

    transform.localRotation = baseline * IMUrotation;
    // print(IMUrotation.eulerAngles.x.ToString() + "," + IMUrotation.eulerAngles.y.ToString() + "," + IMUrotation.eulerAngles.z.ToString());

    if(0!=left.ToInt32())
    {
      //  liveVedioFromWebCamera.LoadRawTextureData(leftData);
      // liveVedioFromWebCamera.LoadImage(leftData);
      Debug.Log("width: "+liveVedioFromWebCamera.width);
      Debug.Log("height: " + liveVedioFromWebCamera.height);
      liveVedioFromWebCamera.LoadRawTextureData(left, (int)size);
      liveVedioFromWebCamera.Apply();
      transform.FindChild("cameraScene").GetComponent<Renderer>().material.mainTexture = liveVedioFromWebCamera;
 //     leftData.Initialize();
    }


  }

  void OnDestroy()
  {
    print("OnDestroy is called");

    device.Stop();
    device.Dispose();
  }

  void DUOFrameReceived(DUODevice sender, ref DUOFrame pFrameData)
  {
    frameCount++;
   // Debug.Log("Frame Count: " + frameCount.ToString() + "  timeStamp:  " + pFrameData.timeStamp.ToString());
    if (pFrameData.IMUPresent)
    {
      left = pFrameData.leftData;
      right = pFrameData.rightData;
      size = pFrameData.width * pFrameData.height;
      Debug.Log("size: "+ size);
     
      
           if (0!=left.ToInt32())
            { 
              leftData = new byte[size];
              Marshal.Copy(left, leftData, 0, (int)size);
            }
           if(0!=right.ToInt32())
            {
              rightData = new byte[size];
              Marshal.Copy(right, rightData, 0, (int)size);
            }
        
      // byte* ptr = (byte*)(void*)left;
      for (int i = 0; i < pFrameData.IMUSamples; i++)
      {
        /*
        Debug.Log("Sample:  " + (i + 1).ToString());
        Debug.Log("Timestamp: " + pFrameData.IMUData[i].timeStamp.ToString());
        Debug.Log("Acceleration:  " + pFrameData.IMUData[i].accelData[0].ToString() + pFrameData.IMUData[i].accelData[1].ToString() + pFrameData.IMUData[i].accelData[2].ToString());
        Debug.Log("Gyro:  " + pFrameData.IMUData[i].gyroData[0].ToString() + pFrameData.IMUData[i].gyroData[1].ToString() + pFrameData.IMUData[i].gyroData[2].ToString());
        Debug.Log("Temperature: " + pFrameData.IMUData[i].tempData.ToString());
        */
        imu_msg.linear_acceleration[0] = pFrameData.IMUData[i].accelData[0] * 9.80151f;
        imu_msg.linear_acceleration[1] = pFrameData.IMUData[i].accelData[1] * 9.80151f;
        imu_msg.linear_acceleration[2] = pFrameData.IMUData[i].accelData[2] * 9.80151f;

        imu_msg.angular_velocity[0] = (float)deg2rad(pFrameData.IMUData[i].gyroData[0] - imu_msg._gyro_offset.x);
        imu_msg.angular_velocity[1] = (float)deg2rad(pFrameData.IMUData[i].gyroData[1] - imu_msg._gyro_offset.y);
        imu_msg.angular_velocity[2] = (float)deg2rad(pFrameData.IMUData[i].gyroData[2] - imu_msg._gyro_offset.z);

      }
    }
  }

  static void DUODeviceStatusChanged(DUODevice sender, bool isRunning)
  {
    if (isRunning)
      Debug.Log("[START DUO DEVICE]");
    else
      Debug.Log("[STOP DUO DEVICE]");
  }


  double deg2rad(double deg)
  {
    return deg * 3.1415926f / 180.0f;
  }

}
