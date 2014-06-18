using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.IO.Ports;
using System.IO;

public class serv
{
    public static void Main()
    {
        IPAddress ipAd = IPAddress.Parse("10.129.139.24");
        // use local m/c IP address, and 
        // use the same in the client

        /* Initializes the Listener */
        TcpListener myList = new TcpListener(ipAd, 8001);
        myList.Start();

        Console.WriteLine("The server is running at port 8001...");
        Console.WriteLine("The local End point is  :" +
                          myList.LocalEndpoint);
        Console.WriteLine("Waiting for a connection.....");

        byte[] b = new byte[100];
        // Initialize the serial port and open it for communication.
        // Check the device manager setting to get the correct COM
        // port number for your Zigbee module
        SerialPort serialPort;
        serialPort = new SerialPort("COM5", 9600, Parity.None, 8, StopBits.One);
        serialPort.Handshake = Handshake.None;

        while (true)
        {
            try
            {
                Socket s = myList.AcceptSocket();
                Console.WriteLine("Connection accepted from " + s.RemoteEndPoint);
                try
                {
                    serialPort.Open();
                }
                catch (Exception e)
                {
                    Console.Write("Error opening serial port for communication:" + e);
                }
                /* Start Listening at the specified port */
                while (true)
                {
                    int k = s.Receive(b);
                    char c;
                    string str = "";
                    
                    for (int i = 0; i < k; i++)
                    {
                        c = Convert.ToChar(b[i]);
                        str += c;
                    }
                    if (str.CompareTo("End") == 0)
                    {
                        Console.WriteLine("Connection Closed");
                        Console.WriteLine("Waiting for a connection.....");
                        break;
                    }
                }
                serialPort.Close();
                s.Close();
        
            }
            catch (Exception e)
            {
                Console.WriteLine("Error..... " + e.StackTrace);
            }

        }

        myList.Stop();
    }
}

