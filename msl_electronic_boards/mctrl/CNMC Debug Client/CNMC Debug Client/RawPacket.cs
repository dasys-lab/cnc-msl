using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CNMC_Debug_Client
{
    static class RawPacket
    {

        public static Byte[] newRawPacket(Packet p)
        {
            return newRawPacket(p.group, p.command, p.data, p.request);
        }

        public static Byte[] newRawPacket(byte group, byte command, Byte[] data, byte req)
        {
            Byte[] buffer = new Byte[Protocol.MAX_PACKET_SIZE];

            if (data != null && data.Length > Protocol.MAX_DATA_SIZE) return null;
            
            int i = 0; // size counter of new packet

            buffer[i++]  = Protocol.PacketStart; // not in CRC
            buffer[i++]  = group;
            buffer[i++] = command;

            if (data != null)
            {
                for (uint j = 0; j < data.Length; ++j)
                {
                    buffer[i++] = data[j];
                }
            }

            if (req != (byte) Protocol.RequestCmd.None)
            {
                buffer[i++]  = req;
            }

            buffer[i++] = Protocol.PacketEnd; // not in CRC

            Byte[] outBuffer = new Byte[i];

            Array.Copy(buffer, outBuffer, i);

            return outBuffer;
        }

       
        public static String formatPacket(Byte[] rawPacket)
        {
            StringBuilder sb = new StringBuilder();

            for (int i = 0; i < rawPacket.Length; ++i)
            {
                sb.Append(String.Format("{0:x2}", rawPacket[i]));
            }

            return sb.ToString();

        }

        public static Byte[] toBytes(Int16 val1)
        {
            return toBytes((UInt16) val1);
        }

        public static Byte[] toBytes(UInt16 val1)
        {
            return new Byte[] { (Byte) (val1 & 0xFF), (Byte) ((val1 >> 8) & 0xFF) };
        }

        public static Byte[] toBytes(Int32 val1)
        {
            return toBytes((UInt32) val1);
        }

        public static Byte[] toBytes(UInt32 val1)
        {
            return new Byte[] { (Byte)(val1 & 0xFF), (Byte)((val1 >> 8) & 0xFF), (Byte)((val1 >> 16) & 0xFF), (Byte)((val1 >> 24) & 0xFF) };
        }

        public static Byte[] toBytes(Int16 val1, Int16 val2, Int16 val3)
        {
            return toBytes((UInt16) val1, (UInt16) val2, (UInt16) val3);
        }

        public static Byte[] toBytes(UInt16 val1, UInt16 val2, UInt16 val3)
        {
            return new Byte[] { (Byte)(val1 & 0xFF), (Byte)((val1 >> 8) & 0xFF),
                                (Byte)(val2 & 0xFF), (Byte)((val2 >> 8) & 0xFF),
                                (Byte)(val3 & 0xFF), (Byte)((val3 >> 8) & 0xFF) };
        }

        public static Byte[] toBytes(Int32 val1, Int32 val2, Int32 val3)
        {
            return toBytes((UInt32)val1, (UInt32)val2, (UInt32)val3);
        }

        public static Byte[] toBytes(UInt32 val1, UInt32 val2, UInt32 val3)
        {
            return new Byte[] { (Byte)(val1 & 0xFF), (Byte)((val1 >> 8) & 0xFF), (Byte)((val1 >> 16) & 0xFF), (Byte)((val1 >> 24) & 0xFF),
                                (Byte)(val2 & 0xFF), (Byte)((val2 >> 8) & 0xFF), (Byte)((val2 >> 16) & 0xFF), (Byte)((val2 >> 24) & 0xFF),
                                (Byte)(val3 & 0xFF), (Byte)((val3 >> 8) & 0xFF), (Byte)((val3 >> 16) & 0xFF), (Byte)((val3 >> 24) & 0xFF)};
        }

        
        public static UInt16 toUInt16(Byte[] bytes, int offset)
        {
            return (UInt16)(((UInt16)bytes[offset]           & (UInt16)0x00FF)
                         | (((UInt16)bytes[offset + 1] << 8) & (UInt16)0xFF00));
        }

        public static Int16 toInt16(Byte[] bytes, int offset)
        {
            return (Int16)toUInt16(bytes, offset);
        }

        public static UInt32 toUInt32(Byte[] bytes, int offset)
        {
            return (UInt32)(((UInt32)(bytes[offset])           & (UInt32)0x000000FF)
                          | ((UInt32)(bytes[offset + 1] << 8)  & (UInt32)0x0000FF00)
                          | ((UInt32)(bytes[offset + 2] << 16) & (UInt32)0x00FF0000)
                          | ((UInt32)(bytes[offset + 3] << 24) & (UInt32)0xFF000000));
        }

        public static Int32 toInt32(Byte[] bytes, int offset)
        {
            return (Int32)toUInt32(bytes, offset);
        }
    }
}
