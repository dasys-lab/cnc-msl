package de.lab.main;

import gnu.io.PortInUseException;
import gnu.io.UnsupportedCommOperationException;
import de.lab.net.Communication;
import de.lab.net.ReaderWriter;
import de.lab.ui.MainWindow;

public class Main {

	static String port = "/dev/ttyUSB2";
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		for(int i = 0; i < args.length; i++)
		{
			if("-gui".equals(args[i]))
			{
				MainWindow window = new MainWindow();
				window.initialize();
			}
			
			if("-autocal".equals(args[i]))
			{
				if((i+2) < args.length)
				{
					autoCalibrate(Integer.valueOf(args[i+1]), Integer.valueOf(args[i+2]));
				}
				
				else
				{
					System.out.println("Usage: visualizer -autocal value1 value2");
				}
			}
			
			if("-port".equals(args[i]))
			{
				if((i+1) < args.length)
				{
					port = args[i+1];
					System.out.println(port);
				}
				
				else
				{
					System.out.println("Usage: -port value");
				}
			}
		}
	}

	private static void autoCalibrate(int range, int numberOfMeasurements) {
		// TODO Auto-generated method stub
		Communication communication = new Communication(port);
		try {
			communication.connect();
			ReaderWriter reader = new ReaderWriter(communication.getPort(), null, null, null, numberOfMeasurements);
			reader.writeData(3);
			reader.writeData(range);
			reader.writeData(numberOfMeasurements);
			
		} catch (PortInUseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedCommOperationException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
