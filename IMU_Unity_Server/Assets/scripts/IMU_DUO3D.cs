using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using DUODeviceLib;


public class IMU_DUO3D : MonoBehaviour
{
  public struct IMU_SENSOR
  {
    public Vector3 linear_acceleration;
    public Vector3 angular_velocity;
    public Vector3 magn;
  }

  IMU_SENSOR imu_msg;

  public Vector3 _gyro_offset = new Vector3(-1.8966f, 1.9159f, 0.1865f);
  public Vector3 _gyro_varience = new Vector3(0.0351f, 0.0149f, 0.0000001f);

  byte[] recBuf = new byte[100];
  byte[] sendBuf = new byte[1024];

  Quaternion IMUrotation;
  bool updateFlag = false;

  Thread receiveThread;
  Thread sendThread;

  Socket socket;

  EndPoint remoteEP;
  EndPoint localEp;

  IntPtr left;

  bool exitSignal = false;//exit UDP send thread and UDP receive thread
  DUODevice device = new DUODevice();

  static int frameCount = 0;

  void DUOFrameReceived(DUODevice sender, ref DUOFrame pFrameData)
  {
    frameCount++;
    Debug.Log("Frame Count: "+ frameCount.ToString()+"  timeStamp:  "+ pFrameData.timeStamp.ToString());
    if (pFrameData.IMUPresent)
    {
      left = pFrameData.leftData;
      
      for (int i = 0; i < pFrameData.IMUSamples; i++)
      {
        Debug.Log("Sample:  " + (i + 1).ToString());
        Debug.Log("Timestamp: " + pFrameData.IMUData[i].timeStamp.ToString());
        Debug.Log("Acceleration:  " + pFrameData.IMUData[i].accelData[0].ToString() + pFrameData.IMUData[i].accelData[1].ToString() + pFrameData.IMUData[i].accelData[2].ToString());
        Debug.Log("Gyro:  " + pFrameData.IMUData[i].gyroData[0].ToString() + pFrameData.IMUData[i].gyroData[1].ToString() + pFrameData.IMUData[i].gyroData[2].ToString());
        Debug.Log("Temperature: " + pFrameData.IMUData[i].tempData.ToString());

        imu_msg.linear_acceleration[0] = pFrameData.IMUData[i].accelData[0] * 9.80151f;
        imu_msg.linear_acceleration[1] = pFrameData.IMUData[i].accelData[1] * 9.80151f;
        imu_msg.linear_acceleration[2] = pFrameData.IMUData[i].accelData[2] * 9.80151f;
        
        imu_msg.angular_velocity[0] = (float)deg2rad(pFrameData.IMUData[i].gyroData[0] - _gyro_offset.x);
        imu_msg.angular_velocity[1] = (float)deg2rad(pFrameData.IMUData[i].gyroData[1] - _gyro_offset.y);
        imu_msg.angular_velocity[2] = (float)deg2rad(pFrameData.IMUData[i].gyroData[2] - _gyro_offset.z);

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

  void Start()
  {
    socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
    remoteEP = new IPEndPoint(IPAddress.Any, 0);
    localEp = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 8888);
    socket.Bind(localEp);

    receiveThread = new Thread(new ThreadStart(UDPReceiveData));
    receiveThread.IsBackground = true;
    //receiveThread.Start();

    sendThread = new Thread(new ThreadStart(UDPSendData));
    sendThread.IsBackground = true;
  //  sendThread.Start();

    //initialize DUO3D camara
    
    device.DUODeviceStatusChanged += DUODeviceStatusChanged;
    device.DUOFrameReceived += DUOFrameReceived;
    device.Resolution = new DUOResolutionInfo() {
      width=320,
      height=240,
      binning = DUOBinning.DUO_BIN_HORIZONTAL2 | DUOBinning.DUO_BIN_VERTICAL2,
      fps=30
    };
    device.Start();
    device.Exposure = 100;
    device.Gain = 10;

    imu_msg.magn = Vector3.zero;

    Debug.Log("inited");

  }

  private void UDPReceiveData()
  {
    while (!exitSignal)
    {
      int dataSize = socket.ReceiveFrom(recBuf, ref remoteEP);
      updateFlag = true;
      float q0 = BitConverter.ToSingle(recBuf, 0);
      float q1 = BitConverter.ToSingle(recBuf, sizeof(float));
      float q2 = BitConverter.ToSingle(recBuf, sizeof(float) * 2);
      float q3 = BitConverter.ToSingle(recBuf, sizeof(float) * 3);
      char end = BitConverter.ToChar(recBuf, sizeof(float) * 4);
      // Debug.Assert((end=='E'),"End of package");
      IMUrotation = new Quaternion(q0, q1, q2, q3);
    }
    Debug.Log("UDPReceiveData thread eixt");
  }

  private void UDPSendData()
  {
    while (!exitSignal)
    {
      if (updateFlag)//obtained the remote address
      {
        Buffer.BlockCopy(BitConverter.GetBytes(_gyro_offset.x), 0, sendBuf, 0 * sizeof(float), sizeof(float));
        Buffer.BlockCopy(BitConverter.GetBytes(_gyro_offset.y), 0, sendBuf, 1 * sizeof(float), sizeof(float));
        Buffer.BlockCopy(BitConverter.GetBytes(_gyro_offset.z), 0, sendBuf, 2 * (sizeof(float)), sizeof(float));
        Buffer.BlockCopy(BitConverter.GetBytes(_gyro_varience.x), 0, sendBuf, 3 * sizeof(float), sizeof(float));
        Buffer.BlockCopy(BitConverter.GetBytes(_gyro_varience.y), 0, sendBuf, 4 * sizeof(float), sizeof(float));
        Buffer.BlockCopy(BitConverter.GetBytes(_gyro_varience.z), 0, sendBuf, 5 * sizeof(float), sizeof(float));
        Buffer.BlockCopy(BitConverter.GetBytes('E'), 0, sendBuf, 6 * sizeof(float), sizeof(char));
        socket.SendTo(sendBuf, sendBuf.Length, SocketFlags.None, remoteEP);
      }
    }
    Debug.Log("UDPSendData thread exit");
  }

  Quaternion baseline = Quaternion.Inverse(Quaternion.identity);

  // Update is called once per frame
  void Update()
  {
    if (Input.GetKeyUp(KeyCode.Space))
    {
      baseline = Quaternion.Inverse(IMUrotation);
    }
    Transform scene = transform.FindChild("cameraScene");
    

    if (updateFlag)
    {
      transform.localRotation = baseline * IMUrotation;
      print(IMUrotation.eulerAngles.x.ToString() + "," + IMUrotation.eulerAngles.y.ToString() + "," + IMUrotation.eulerAngles.z.ToString());
      IMUrotation = Quaternion.identity;
      updateFlag = false;
    }
  }

  void OnDestroy()
  {
    print("OnDestroy is called");
    socket.Close();
    exitSignal = true;
    //client.Close ();
    receiveThread.Abort();
    sendThread.Abort();
    device.Stop();
    device.Dispose();
  }

  double deg2rad(double deg)
  {
    return deg * 3.1415926f / 180.0f;
  }

}
