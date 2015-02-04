using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CNMC_Debug_Client
{
    class Packet
    {
        public Byte   group;
        public Byte   command;
        public Byte   request;
        public Byte[] data;
        public Byte[] rawPacket;

        public Packet()
        {
        }

        public Packet(Byte[] rawPacket) : this(rawPacket, rawPacket.Length)
        {
        }

        public Packet(Byte[] rawPacket, int length)
        {
            group = rawPacket[Protocol.GROUP_POS];
            command = rawPacket[Protocol.COMMAND_POS];
            request = rawPacket[Protocol.REQUEST_REL_POS + length];

            int dataLength = Protocol.DATA_REL_END_POS + length;

            data = new Byte[dataLength];

            Array.Copy(rawPacket, Protocol.DATA_START_POS, data, 0, dataLength);

            this.rawPacket = rawPacket;
        }

    }
}
