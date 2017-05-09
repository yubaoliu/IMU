using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;  
using System.Net.Sockets;
using System.Net;
using System.Threading;
public class IMU_DUO3D : MonoBehaviour {
	Thread receiveThread;
	Socket socket;
	//UdpClient client;
	EndPoint remoteEP;
	EndPoint localEp;
	byte[] recBuf = new byte[100];
  Quaternion iMUrotatation;
	//Vector3 eulerAngle;
	bool updataFlag=false;
//	Stopwatch stopwatch = new Stopwatch();
	// Use this for initialization
	void Start () {
		UnityEngine.Debug.Log ("IMU Behaviour starting");


		socket = new Socket (AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
		//client = new UdpClient ();
		remoteEP = new IPEndPoint (IPAddress.Any, 0);
		localEp = new IPEndPoint (IPAddress.Parse ("127.0.0.1"), 8888);
		//client.Client.Bind (localEp);
		socket.Bind(localEp);

		receiveThread = new Thread (new ThreadStart (UDPReceiveData));
		receiveThread.IsBackground = true;
		receiveThread.Start ();
		Debug.Log ("inited");
		
	}
	private void UDPReceiveData()
	{
		while (true) {
      int dataSize = socket.ReceiveFrom (recBuf, ref remoteEP);
			updataFlag = true;
      float q0 = BitConverter.ToSingle(recBuf, 0);
      float q1 = BitConverter.ToSingle(recBuf, sizeof(float));
      float q2 = BitConverter.ToSingle(recBuf, sizeof(float) * 2);
      float q3 = BitConverter.ToSingle(recBuf, sizeof(float) * 3);
      char end = BitConverter.ToChar(recBuf, sizeof(float) * 4);
      Debug.Assert((end=='E'),"End of package");
      iMUrotatation = new Quaternion(q0, q1, q2, q3);
	//		Debug.Log ("Received data");
  /*
			float euX = BitConverter.ToSingle (recBuf, 0);
			float euY = BitConverter.ToSingle (recBuf, sizeof(float));
			float euZ=BitConverter.ToSingle(recBuf,sizeof(float)*2);

      iMUrotatation = Quaternion.Euler(euX, euY, euZ);
  		print("X:"+euX.ToString ()+" Y:"+euY.ToString()+" Z:"+euZ.ToString()+'\n');
      */
		}
	}
/*	private void SocketSend(string sendStr)  
	{  
	//	string editString="hello wolrd";  
		byte[] sendData=new byte[1024];  
		sendData=Encoding.ASCII.GetBytes(sendStr);  
	//	socket.SendTo(sendData,sendData.Length,SocketFlags.None,ipEnd);  
	}
	*/
	// Update is called once per frame
	void Update () {
		//Quaternion oldOri = transform.localRotation;
		//if(!oldOri.Equals(iMUrotatation))
		if (updataFlag) {
			transform.localRotation = iMUrotatation;
      //transform.Rotate (iMUrotatation.eulerAngles);
      iMUrotatation = Quaternion.identity;
			//	Quaternion tmp=new Quaternion();
			//	tmp.eulerAngles = eulerAngle;
			//	transform.rotation = tmp;
			print (iMUrotatation.eulerAngles.x.ToString () +","+ iMUrotatation.eulerAngles.y.ToString () +","+ iMUrotatation.eulerAngles.z.ToString ());
			updataFlag = false;
		}
	}
	void OnDestroy(){
		print ("OnDestroy is called");
		socket.Close ();
		//client.Close ();
		receiveThread.Abort ();
	}
}
