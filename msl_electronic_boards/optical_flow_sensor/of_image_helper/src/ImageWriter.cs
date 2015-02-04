using System;
using System.IO;
using System.IO.Ports;
using System.Threading;
using System.Collections.Generic;

namespace OpticalFlowSensor
{

	public class ImageWriter
	{
	    static bool running;
	    static SerialPort serialPort;
	
	    public static void Main()
	    {
	        string message;
	        StringComparer stringComparer = StringComparer.OrdinalIgnoreCase;
	        Thread readThread = new Thread(Read);
	
	        // Create a new SerialPort object with default settings.
	        serialPort = new SerialPort();
	
	        // Allow the user to set the appropriate properties.
	        serialPort.PortName = "/dev/ttyUSB0";
	        serialPort.BaudRate = 9600;
	        serialPort.Parity = Parity.None;
	        serialPort.DataBits = 8;
	        serialPort.StopBits = StopBits.One;
	        serialPort.Handshake = Handshake.None;
			serialPort.ReadBufferSize = 901;
			
	        // Set the read/write timeouts
	        serialPort.ReadTimeout = 500;
	        serialPort.WriteTimeout = 500;
	
	        serialPort.Open();
	        running = true;
	        readThread.Start();
	
	        Console.WriteLine("Type QUIT to exit");
	
	        while (running)
	        {
	            message = Console.ReadLine();
	
	            if (stringComparer.Equals("quit", message))
	            {
	                running = false;
	            }
				Thread.Sleep(1);
	        }
	
	        readThread.Join();
	        serialPort.Close();
	    }
	
	    public static void Read()
	    {
			string image = "image_";
			int counter = 0;
			string type = ".csv";

			List<string> buffer = new List<string>();
			
	        while (running)
	        {
	            try
	            {
	                string message = serialPort.ReadLine();
					//Console.WriteLine(message);
					
					int indexOf = message.IndexOf('-');
					
					if( indexOf == 0 )
					{
						if( buffer.Count == 30 )
						{
							Console.WriteLine("Filecounter : " + counter);
							WriteBufferToFile(image + counter + type,buffer);
							counter++;
						}
						buffer.Clear();
					}
					else
					{
						buffer.Add(message);
					}
					
//					for(int i=0; i<message.Length; i++)
//					{
//						buffer.Add( Convert.ToString(message[i]) );	
//					}
//	                
//					Console.WriteLine("message count : " + message.Length);
//					if( buffer.Count > 901 )
//					{
//						Console.WriteLine("buffer val : " + buffer[900]);	
//					}
//					
//					Console.WriteLine("counts : " + buffer.Count);
//					int frameCount = 0;
//					foreach(string b in buffer)
//					{
//						if( b == "---" )
//						{
//							frameCount++;
//						}
//					}
//					Console.WriteLine("frames : " + frameCount);
	            }
	            catch (TimeoutException) { }
	        }
	    }
		
		public static void WriteBufferToFile(string filename, List<string> buffer)
		{
			StreamWriter file = new StreamWriter(filename);
			
			foreach(string s in buffer)
			{
				string tmp = s.Replace(" ",";");
				file.WriteLine(tmp);	
			}
			
			file.Flush();
			file.Close();

		}
	
	
	}
}