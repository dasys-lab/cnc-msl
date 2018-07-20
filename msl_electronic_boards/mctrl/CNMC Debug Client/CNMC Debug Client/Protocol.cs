using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CNMC_Debug_Client
{
    static class Protocol
    {
        public enum Group : byte
        {
            Configure = (0x50),
            ConfigureResponse = (0x51),
            Control = (0x52),
            Request = (0x54),
            RequestResponse = (0x55),
            ControlConfig = (0x56),
            ControlConfigResponse = (0x57),
            ErrorResponse = (0x59),
        }

        public enum ConfigureCmd : byte
        {
            SetMode = (0x10),
            SetCycleTime = (0x11),
            CurrentLimitOn = (0x12),
            CommandTimeout = (0x13),
            MaxCurrent = (0x20),
            MaxRPM = (0x21),
            NominalCurrent = (0x22),
            NominalRPM = (0x23),
            GearRatio = (0x30),
            WheelRadius = (0x31),
            TicksPerRotation = (0x33),
            MotorTorque = (0x34),
            MotorDirection = (0x35),
            MotorMaxTemp = (0x40),
            MotorNominalTemp = (0x41),
            EnvironmentTemp = (0x42),
            MotorWindingTemp = (0x43),
            MotorWindingGN = (0x44),
            MotorWindingGT = (0x45),
            MotorChassisTemp = (0x46),
            MotorChassisGN = (0x47),
            MotorChassisGZ = (0x48),
            SaveConfig = (0x50),
            LoadConfig = (0x51),
            IoPortConf = (0x60),
            RobotRadius = (0x70),
            WheelAngle = (0x71),
        }

        public enum ControlCmd : byte
        {
            SetAllPWM = (0x10),
            SetPWM = (0x11),
            SetAllDirection = (0x12),
            SetDirection = (0x13),
            SetAllRPM = (0x20),
            SetRPM = (0x21),
            SetMotionVector = (0x30),
            ResetHard = (0x40),
            ResetController = (0x41),
        }

        public enum RequestCmd : byte
        {
            None     = (0x00),
            MotorRPM = (0x10),
            MotorPWM = (0x11),
            MotorVoltage = (0x12),
            MotorCurrent = (0x13),
            MotorTemp = (0x14),
            MotorTorque = (0x15),
            MotorPower = (0x16),
            EncoderRelative = (0x20),
            EncoderAbsolute = (0x21),
            SupplyVoltage = (0x30),
            ReadIO = (0x40),
            SetIO = (0x41),
            ReadAnalog = (0x42),
            MotionVector = (0x60),
            AverageSleepTime = (0x61),
            PathVector = (0x62),
        }

        public enum ControlConfigCmd : byte
        {
            PidKp = (0x10),
            PidKi = (0x11),
            PidB = (0x12),
            PidKd = (0x13),
            PidKdi = (0x14),
            ControllerCommit = (0x15),
            DeadBand = (0x2A),
            RotationErrorWeight = (0x31),
            RotationErrorAccelWeight = (0x32),
            RotationErrorVeloWeight = (0x33),
            AccelBoundCurveMin = (0x40),
            AccelBoundCurveMax = (0x41),
            AccelCurveDegree = (0x42),
            FailSafeValues = (0x50),
        }

        public enum ErrorResponse : byte
        {
            UnknownError = (0x01),
            UnknownCommand = (0x02),
            ParameterError = (0x10),
            OutOfRange = (0x11),
            CycleOvertime = (0x20),
        }

        public const Byte PacketStart = 0x81;
        public const Byte PacketEnd = 0x82;
        public const Byte PacketQuote = 0x84;
        public const Byte PacketMaxCounter = 0x83;

        // start    = 1
        // counter  = 1
        // group    = 1
        // command  = 1
        // data     = 12
        // req      = 1
        // crc      = 1
        // end      = 1

        public const int MAX_DATA_SIZE = 12;
        public const int MAX_PACKET_SIZE = 19;

        public const int START_POS = 0;
        public const int GROUP_POS = 1;
        public const int COMMAND_POS = 2;
        public const int DATA_START_POS = 3;

        // these positions are relative to the end of the packet
        public const int DATA_REL_END_POS = -2;
        public const int REQUEST_REL_POS = -1;
        public const int END_REL_POS = 0;

    }
}
