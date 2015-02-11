package de.lab.net;

import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.TooManyListenersException;

import javax.swing.JLabel;

import de.lab.ui.Plotter;

public class ReaderWriter extends Thread implements SerialPortEventListener{

	private SerialPort port;
	private InputStream inputStream;
	private OutputStream outputStream;
	private byte[] buffer;
	private JLabel lblEntfernung;
	private JLabel label;
	private Plotter plotter;
	private ArrayList<Integer> valueList;
	private int counter;
	private boolean gui;
	private int numberOfMeasurements;
	private BufferedWriter file;
	
	public ReaderWriter(SerialPort port, Plotter plotter, JLabel label, JLabel lblEntfernung, int numberOfMeasurements)
	{
		this.port = port;
		this.gui = true;
		try {
			this.port.addEventListener(this);
		} catch (TooManyListenersException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		try {
			this.inputStream = port.getInputStream();
			this.outputStream = port.getOutputStream();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.buffer = new byte[1024];

		this.label = label;
		this.lblEntfernung = lblEntfernung;
		this.plotter = plotter;
		
		if((null == this.lblEntfernung) && (null == this.plotter) && (null == this.label)) this.gui = false;
		this.valueList = new ArrayList<Integer>();
		this.counter = 1;
		this.numberOfMeasurements = numberOfMeasurements;
		try {
			this.file = new BufferedWriter(new FileWriter("./log"));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	@Override
	public void run()
	{
		while(true)
		{
			try {
				sleep(200);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	public void writeData(int data)
	{
		try {
			System.out.println("sende: " + data);
			this.outputStream.write(data);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	@Override
	public void serialEvent(SerialPortEvent event) {
		// TODO Auto-generated method stub
		switch(event.getEventType()) {
        case SerialPortEvent.BI:
        case SerialPortEvent.OE:
        case SerialPortEvent.FE:
        case SerialPortEvent.PE:
        case SerialPortEvent.CD:
        case SerialPortEvent.CTS:
        case SerialPortEvent.DSR:
        case SerialPortEvent.RI:
        case SerialPortEvent.OUTPUT_BUFFER_EMPTY:
            break;
        case SerialPortEvent.DATA_AVAILABLE:
        	int data;
            
            try
            {
                int len = 0;
                while ( ( data = this.inputStream.read()) > -1 )
                {
                    if ( data == '\n' ) {
                        break;
                    }
                    buffer[len++] = (byte) data;
                }
                String value = new String(buffer,0,len);
              	int x_value = Integer.valueOf(value);
              	
              	
              	if(!gui)
              	{
              		if(counter > 32)
              		{
              			if(file != null)
              			{
              				file.flush();
              				file.close();
              			}
              			
              			System.exit(0);
              		}
              		
              		if(valueList.size() == numberOfMeasurements)
                  	{
                  		double mean = 0.0;
                  		
                  		for(int i = 0; i < 100; i++)
                  		{
                  			mean += valueList.get(i);
                  		}
                  		
                  		mean /= numberOfMeasurements;
                  		
                  		if(file != null)
                  		{
                  			file.write((counter-1) + " " + mean);
                  			file.newLine();
                  		}
                  		
                  		System.out.println("run " + counter + "\nmean: " + mean + "cm");
                  		
                  		valueList.clear();
                  		counter++;
                  	}
                  	
                  	else
                  	{
                  		valueList.add(x_value);
                  	}
              	}
              	
              	else
              	{
              		//lblObjekt.setLocation(lblSensor.getX() + lblSensor.getWidth() + x_value, lblObjekt.getY());
              		if(plotter.getData().size() == 20)
              		{
              			plotter.getData().remove(0);
              		}
              		
              		else
              		{
              			plotter.getData().add(x_value);
              			label.repaint();
              		}
              		
              		lblEntfernung.setText("Entfernung: " + x_value + "cm");
              	}
            }
            catch ( IOException e )
            {
                e.printStackTrace();
                System.exit(-1);
            }      
            break;
        }

	}

}
