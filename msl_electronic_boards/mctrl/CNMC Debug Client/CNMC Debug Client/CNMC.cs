using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.Threading;

namespace CNMC_Debug_Client
{
    public class CNMC
    {

        private SerialPort port;
        private GUI gui;

        public CNMC(String portString)
        {

            this.port = new SerialPort(portString, 57600, Parity.None, 8, StopBits.One);
            //this.port = new SerialPort(portString, 625000, Parity.None, 8, StopBits.One);
            port.Open();

        }

        public void setGui(GUI newGUI)
        {
            this.gui = newGUI;
            new Thread(new ThreadStart(endlessPortReader)).Start();
            new Thread(new ThreadStart(periodicSender)).Start();
            new Thread(new ThreadStart(endlessMessageDispatcher)).Start();
        }

        private void endlessPortReader()
        {

            Byte[] packet = new Byte[Protocol.MAX_PACKET_SIZE * 2];
            int pi = 0;
            StringBuilder sb = new StringBuilder();
            Byte b;

            bool inPacket = false;
            bool quoted = false;

            while (true)
            {
                b = (Byte) port.ReadByte();

                //gui.addNoticeMessage(String.Format("{0:x2}", b));

                if (b == Protocol.PacketStart)
                {
                    //gui.addNoticeMessage("Packet start");
                    inPacket = true;
                }

                if (inPacket)
                {
                    sb.Append(String.Format("{0:x2}", b));
                    //gui.addNoticeMessage(String.Format("{0:x2}", b));

                    if (b == Protocol.PacketQuote && quoted == false)
                    {
                        gui.addNoticeMessage("Quote");
                        quoted = true;
                        continue;
                    }

                    packet[pi++] = b;

                    if (b == Protocol.PacketEnd && quoted == false)
                    {
                        //gui.addNoticeMessage("Packet end");
                        inPacket = false;
                        Packet p = new Packet(packet, pi);
                        pi = 0;

                        gui.addReceiveMessage(sb.ToString());
                        sb.Length = 0;

                        decodeCommand(p);

                    }

                }
                else
                {
                    // not inside a packet
                    if (b == 0x0a)
                    {
                        gui.addReceiveMessage(sb.ToString());
                        sb.Length = 0;
                    }
                    else
                    {
                        sb.Append((char)b);
                    }
                }

                quoted = false;
            }
        }

        private void periodicSender()
        {
            while (true)
            {
                if (gui.isSendPeriodicPWM()) gui.pwm_send_pwm();
                if(gui.isUpdateVoltage()) requestVoltage();
                if(gui.isUpdateCurrent()) requestCurrent();
                if(gui.isUpdateTicks()) requestTicksRel();

                Thread.Sleep(250);
            }
        }

        private Queue<Byte[]> messageQueue = new Queue<Byte[]>();

        private void endlessMessageDispatcher()
        {
            StringBuilder sb = new StringBuilder();
            Byte[] packet;

            while (true)
            {
                Monitor.Enter(messageQueue);

                if (messageQueue.Count == 0)
                {
                    Monitor.Wait(messageQueue);
                }

                packet = messageQueue.Dequeue();

                Monitor.Exit(messageQueue);

                sb.Length = 0;

                for (int i = 0; i < packet.Length; ++i)
                {
                    sb.Append(String.Format("{0:x2}", packet[i]));
                }

                gui.addSendMessage(sb.ToString());

                // send the quoted packet
                for (int i = 0; i < packet.Length; ++i)
                {
                    port.Write(packet, i, 1);
                    //Thread.Sleep(1);
                }
            }
        }

        private void decodeCommand(Packet p)
        {
            switch (p.group)
            {
                case (byte)Protocol.Group.RequestResponse:
                    switch (p.command)
                    {
                        case (byte)Protocol.RequestCmd.MotorPWM:
                            gui.updatePWM(RawPacket.toInt16(p.data, 0), RawPacket.toInt16(p.data, 2), RawPacket.toInt16(p.data, 4));
                            break;

                        case (byte)Protocol.RequestCmd.MotorCurrent:
                            gui.updateCurrent(RawPacket.toUInt16(p.data, 0), RawPacket.toUInt16(p.data, 2), RawPacket.toUInt16(p.data, 4));
                            break;

                        case (byte)Protocol.RequestCmd.SupplyVoltage:
                            gui.updateSupplyVoltage(RawPacket.toUInt16(p.data, 0));
                            break;

                        case (byte)Protocol.RequestCmd.EncoderRelative:
                            gui.updateEncoderRelative(RawPacket.toInt32(p.data, 0), RawPacket.toInt32(p.data, 4), RawPacket.toInt32(p.data, 8));
                            break;

                        default:
                            gui.addNoticeMessage("Decode error: RequestResponse");
                            gui.addNoticeMessage(RawPacket.formatPacket(p.rawPacket));
                            break;
                    }
                    break;

                case (byte)Protocol.Group.ErrorResponse:
                    switch (p.command)
                    {
                        case (byte)Protocol.ErrorResponse.CycleOvertime:
                            gui.addNoticeMessage("Error: Overtime");
                            break;

                        case (byte)Protocol.ErrorResponse.OutOfRange:
                            gui.addNoticeMessage("Error: Out Of Range");
                            break;

                        case (byte)Protocol.ErrorResponse.ParameterError:
                            gui.addNoticeMessage("Error: Parameter Error");
                            break;

                        case (byte)Protocol.ErrorResponse.UnknownError:
                            gui.addNoticeMessage("Error: Unknown Error");
                            break;

                        case (byte)Protocol.ErrorResponse.UnknownCommand:
                            gui.addNoticeMessage("Error: Unknown Command");
                            break;

                        default:
                            gui.addNoticeMessage("Decode error: ErrorResponse Packet");
                            gui.addNoticeMessage(RawPacket.formatPacket(p.rawPacket));
                            break;
                    }
                    break;
                default:
                    gui.addNoticeMessage("Unknown Group");
                    gui.addNoticeMessage(RawPacket.formatPacket(p.rawPacket));
                    break;
            }
        }

        public void requestAllPWM()
        {
            Byte[] rawPacket = RawPacket.newRawPacket((byte)Protocol.Group.Request, (byte)Protocol.RequestCmd.MotorPWM, null, 0);

            sendPacket(rawPacket);
        }

        public void setAllPWM(Int16 pwm1, Int16 pwm2, Int16 pwm3)
        {

            Byte[] data = RawPacket.toBytes(pwm1, pwm2, pwm3);

            Byte[] rawPacket = RawPacket.newRawPacket((byte)Protocol.Group.Control, (byte)Protocol.ControlCmd.SetAllPWM, data, 0);

            sendPacket(rawPacket);
        }

        public void setMaxCurrent(Byte current)
        {
            Byte[] data = { current };

            Byte[] rawPacket = RawPacket.newRawPacket((byte)Protocol.Group.Configure, (byte)Protocol.ConfigureCmd.MaxCurrent, data, 0);

            sendPacket(rawPacket);
        }

        public void requestVoltage()
        {
            Byte[] rawPacket = RawPacket.newRawPacket((byte)Protocol.Group.Request, (byte)Protocol.RequestCmd.SupplyVoltage, null, 0);

            sendPacket(rawPacket);
        }

        public void requestCurrent()
        {
            Byte[] rawPacket = RawPacket.newRawPacket((byte)Protocol.Group.Request, (byte)Protocol.RequestCmd.MotorCurrent, null, 0);

            sendPacket(rawPacket);
        }

        public void requestTicksRel()
        {
            Byte[] rawPacket = RawPacket.newRawPacket((byte)Protocol.Group.Request, (byte)Protocol.RequestCmd.EncoderRelative, null, 0);

            sendPacket(rawPacket);
        }

        public void setCycleTime(Byte msec)
        {
            Byte[] data = { msec };
            Byte[] rawPacket = RawPacket.newRawPacket((byte)Protocol.Group.Configure, (byte)Protocol.ConfigureCmd.SetCycleTime, data, 0);

            sendPacket(rawPacket);
        }

        private void sendPacket(Byte[] packet)
        {
            Byte[] outBuffer = new Byte[packet.Length * 2];
            int bi = 0; // buffer index

            // copy and quote the packet
            for (int i = 0; i < packet.Length; ++i)
            {

                // quote only the payload
                if (i > 3 && i < packet.Length - 1)
                {
                    switch (packet[i])
                    {
                        case Protocol.PacketEnd:
                        case Protocol.PacketQuote:
                        case Protocol.PacketStart:
                            // quote the value
                            outBuffer[bi++] = Protocol.PacketQuote;
                            break;

                        default:
                            break;
                    }
                }

                // copy the value
                outBuffer[bi++] = packet[i];
            }

            Byte[] tmpBuffer = new Byte[bi];

            System.Array.Copy(outBuffer, tmpBuffer, bi);

            Monitor.Enter(messageQueue);

            messageQueue.Enqueue(tmpBuffer);

            Monitor.PulseAll(messageQueue);

            Monitor.Exit(messageQueue);

        }

    }
}
