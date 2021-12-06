using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;

public class ClientController : MonoBehaviour
{

    private IPEndPoint ipEndPoint;
    private UdpClient udpClient;
    private byte[] sendByte;

    void Start()
    {
        ipEndPoint = new IPEndPoint(IPAddress.Parse("192.168.11.3"), 2000);
        udpClient = new UdpClient();
        Debug.Log("Client_start");
        SendUDPData("hello_server");
    }

    void Update()
    {
        SendUDPData("hello_server");
    }

    public void SendUDPData(string tempData)
    {
        sendByte = System.Text.Encoding.UTF8.GetBytes(tempData);
        udpClient.Send(sendByte, sendByte.Length, ipEndPoint);
    }
}