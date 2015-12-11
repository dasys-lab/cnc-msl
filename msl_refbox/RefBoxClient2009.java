/**
  This file is part of the MSL Refbox.
  
  The MSL Refbox is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License.
  
  The MSL Refbox is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with the MSL Refbox.  If not, see <http://www.gnu.org/licenses/>.
*/

package org.robocup.msl.refbox.applications.refboxTestClient2009;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ConnectException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import javax.swing.JFrame;

import org.robocup.msl.refbox.ConfigFileReader;
import org.robocup.msl.refbox.data.ConfigurationData;
import org.robocup.msl.refbox.data.ConfigurationDataCommunication;
import org.robocup.msl.refbox.protocol.Protocol2009;

public class RefBoxClient2009 {
    private static final int NTHREADS = 20;
    private static final Executor EXECUTOR = Executors.newFixedThreadPool(NTHREADS);

    private RefboxClientApplication tca = null;
    private String multiCastAddress;
    private boolean refboxMulticastProtocol2009;
    private int portUsedByRefbox;

    public RefBoxClient2009() {
        // empty
    }

    public RefboxClientApplication getTca() {
        return this.tca;
    }

    public void createRefBoxApplication() {
        this.tca = new RefboxClientApplication();
        this.tca.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.tca.setVisible(true);
    }

    /**
     * @param args
     * @throws Exception 
     */
    public static void main(final String[] args) {
        final RefBoxClient2009 rbm = new RefBoxClient2009();
        rbm.start();
    }

    public void start() {
        final ConfigFileReader cfr = new ConfigFileReader();
        final ConfigurationData configData = cfr.read();
        for (ConfigurationDataCommunication cdc : configData.getDataCommunication()) {
            if (cdc.isSender() && cdc.getProtocol().equalsIgnoreCase(Protocol2009.class.getSimpleName())) {
                // data how refbox sends protocol 2009 (first entry)
                if (cdc.isMulticast()) {
                    this.multiCastAddress = cdc.getMulticastAddress();
                    this.refboxMulticastProtocol2009 = true;
                }
                else {
                    this.refboxMulticastProtocol2009 = false;
                }
                this.portUsedByRefbox = cdc.getServerPort();
            }
        }

        Runnable guiTask = new Runnable() {
            public void run() {
                createRefBoxApplication();
            }
        };
        Runnable retrieveTask = new Runnable() {
            public void run() {
                retrieveData();
            }
        };
        EXECUTOR.execute(guiTask);
        EXECUTOR.execute(retrieveTask);
    }

    public void retrieveData() {
        int currentRetry = 0;
        while (currentRetry < 10) {
            try {
                while ((this.tca == null) || !this.tca.isVisible()) {
                    Thread.yield();
                }
                if (this.refboxMulticastProtocol2009) {
                    handleMulticastProtocol();
                }
                else {
                    handlePointToPointProtocol();
                }
            }
            catch (final ConnectException e) {
                currentRetry++;
            }
            catch (final Throwable t) {
                t.printStackTrace();
                currentRetry++;
            }
        }
        this.tca.newData("FINISHED to listen !!\n");
    }
    
    private void handlePointToPointProtocol() throws UnknownHostException, IOException {
        this.tca.newData("start point-to-point connection for protocol 2009 at port " 
                        + this.portUsedByRefbox + " on localhost\n");
        final Socket sock = new Socket("localhost", this.portUsedByRefbox);
        sock.setReuseAddress(true);
        sock.setKeepAlive(true);
        sock.setTcpNoDelay(true);
        this.tca.newData("start reading from socket\n");
        final InputStreamReader isr = new InputStreamReader(sock.getInputStream());
        final BufferedReader br = new BufferedReader(isr);

        while (true) {
            final char[] buf = new char[2048];
            final int charsReceived = br.read(buf);
            // Finally, let us do something useful with the data we just received,
            // like print it on stdout :-)
            System.out.println("Received data from: " + sock.getInetAddress().getHostAddress() + ":" 
                            + sock.getPort()
                            + " with length: " + charsReceived);
            final String line = new String(buf, 0, charsReceived);
            System.out.println(line);
            this.tca.newData("received = [" + line + "]\n");
        }
    }

    private void handleMulticastProtocol() throws IOException, UnknownHostException {
        this.tca.newData("start multicast connection for protocol 2009 on port: " 
                        + this.portUsedByRefbox + "\n");

        final MulticastSocket multiCastSocket = new MulticastSocket(this.portUsedByRefbox);

        // join the multicast group
        multiCastSocket.joinGroup(InetAddress.getByName(this.multiCastAddress));
        while (true) {
            final byte[] buf = new byte[2048];
            final DatagramPacket pack = new DatagramPacket(buf, buf.length);

            multiCastSocket.receive(pack);

            // Finally, let us do something useful with the data we just received,
            // like print it on stdout :-)
            System.out.println("Received data from: " + pack.getAddress().toString() + ":" + pack.getPort()
                            + " with length: " + pack.getLength());
            System.out.write(pack.getData(), 0, pack.getLength());
            System.out.println();
            final String line = new String(pack.getData(), 0, pack.getLength());
            this.tca.newData("received = [" + line + "]\n");
        }
    }

    public void dispose() {
        this.tca.dispose();
    }
}
