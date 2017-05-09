using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Collections;
using System.Diagnostics;
using System.Threading;
public class IMU_JY901BT : MonoBehaviour
{

  // Use this for initialization
  UdpClient client;
  IPEndPoint localEp;
  IPEndPoint remoteEp;
  //Stopwatch stopwatch = new Stopwatch();
  Quaternion quat;
  Thread receiveThread;
  void Start()
  {
    UnityEngine.Debug.Log("starting");
    client = new UdpClient();   //create UDP multicast client

    localEp = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 8888);
    remoteEp = new IPEndPoint(IPAddress.Any, 8888);
    client.Client.Bind(localEp);

    receiveThread = new Thread(new ThreadStart(UDPReceiveData));
    receiveThread.IsBackground = true;
    receiveThread.Start();
  }

  // Update is called once per frame
  void Update()
  {
    //stopwatch.Start();

    transform.rotation = quat;   //set camera orientation 
                                                       //stopwatch.Stop();
                                                       //UnityEngine.Debug.Log(stopwatch.ElapsedMilliseconds);
   // stopwatch.Reset();

  }
  private void UDPReceiveData()
  {
    while(true)
    { 
      Byte[] data = client.Receive(ref remoteEp);  //receive a packet from sender
      int dataSize = data.Length;
      float q0 = BitConverter.ToSingle(data, 0);
      float q1 = BitConverter.ToSingle(data, 4);
      float q2 = BitConverter.ToSingle(data, 8);
      float q3 = BitConverter.ToSingle(data, 12);
      byte check = data[16]; // this byte should be 'A' or 65, used for debugging purposes
                             // Vector3 initialPos = new Vector3(18, 1, 34);   // semi random values of initial camera position that work well...
                             //  Vector3 position = new Vector3(-xPos / 10, -yPos / 10, -zPos / 10);   // negate and divide by 10 to correct for units and make it work with Unity
                             // position = position + initialPos; //add position to initial offset
      quat = new Quaternion(q1, -q3, q2, q0); // I have no idea why it works when q2 is swappped with -q3, but this makes the Unity orientation look correct (could be because Unity axes are different than openCV axes)
      }                                       //  transform.position = position; //set camera position 
  }

  void OnDestroy()
  {
    receiveThread.Abort();
  }

}
