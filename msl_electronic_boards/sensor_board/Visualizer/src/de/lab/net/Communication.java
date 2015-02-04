package de.lab.net;

import gnu.io.CommPortIdentifier;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import java.util.Enumeration;

public class Communication {

	private SerialPort port;
	private String portToConnect;
	
	public Communication(String portToConnect)
	{
		this.portToConnect = portToConnect;
	}
	
	@SuppressWarnings("rawtypes")
	public void connect() throws PortInUseException, UnsupportedCommOperationException
	{
		Enumeration portIds = CommPortIdentifier.getPortIdentifiers();
			
		while(portIds.hasMoreElements())
		{
			CommPortIdentifier portIdentifier = (CommPortIdentifier) portIds.nextElement();
            if(portIdentifier.getName().equals(this.portToConnect))
            {
            	this.port = (SerialPort) portIdentifier.open(this.getClass().getName(), 2000);
            	this.port.setSerialPortParams(9600, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
            	this.port.notifyOnDataAvailable(true);
            }
		}
	}
	
	public String getPortTypeName ( int portType )
    {
        switch ( portType )
        {
            case CommPortIdentifier.PORT_I2C:
                return "I2C";
            case CommPortIdentifier.PORT_PARALLEL:
                return "Parallel";
            case CommPortIdentifier.PORT_RAW:
                return "Raw";
            case CommPortIdentifier.PORT_RS485:
                return "RS485";
            case CommPortIdentifier.PORT_SERIAL:
                return "Serial";
            default:
                return "unknown type";
        }
    }


	public SerialPort getPort() {
		return port;
	}
}
