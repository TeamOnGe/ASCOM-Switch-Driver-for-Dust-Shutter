using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.IO.Ports;
using System.Threading;
using ASCOM;
using ASCOM.Astrometry;
using ASCOM.Astrometry.AstroUtils;
using ASCOM.Utilities;
using ASCOM.DeviceInterface;

namespace ASCOM.Shutter
{
    enum ProtocolSpecialChars
    {
        PROT_SPEC_CHAR_START_CHAR = 0x01,               //Start char, Start Of Transmission
        PROT_SPEC_CHAR_END_OF_TRANSMISSION = 0x04,      //End Char, End Of Transmission
        PROT_SPEC_CHAR_ACK = 0x06,                      //Acknowledge
        PROT_SPEC_CHAR_NACK = 0x15,                     //Not Acknowledge
        PROT_SPEC_CHAR_MSN = 0x20,                      //Most significant nibble identifier
        PROT_SPEC_CHAR_LSN = 0x30                       //Least significant nibble identifier
    };

    enum DeviceCategories
    {
        DEV_CAT_MASTER,
        DEV_CAT_TEMPERATURE,
        DEV_CAT_PRESSURE_PROBE,
        DEV_CAT_HUMIDITY_PROBE,
        DEV_CAT_LUMINOSITY_PROBE,
        DEV_CAT_LINEAR_ACTUATOR,
        DEV_CAT_ROTATORY_ACTUATOR,
        DEV_CAT_LINEAR_ENCODER,
        DEV_CAT_ROTATORY_ENCODER,
        DEV_CAT_SHUTTER
    };

    enum ProtocolStates
    {
        PROTOCOL_STATE_IDLE,
        PROTOCOL_STATE_START_RECEIVED,
        PROTOCOL_STATE_ACK_RECEIVED,
        PROTOCOL_STATE_EOT_RECEIVED
    };

    enum ProtocolCommands
    {
        // General commands
        PROTOCOL_CMD_NONE,
        PROTOCOL_CMD_NEW_DEVICE_DISCOVER,
        PROTOCOL_CMD_NEW_DEVICE_REGISTER,
        PROTOCOL_CMD_NEW_DEVICE_ASSIGN_ADDR,
        PROTOCOL_CMD_RESET_ADDR,
        PROTOCOL_CMD_SOFTWARE_RESET,
        PROTOCOL_CMD_CLEAR_MEMORY,

        // Shutter specific commands
        PROTOCOL_CMD_SHUTTER_OPEN,                      // Open the shutter
        PROTOCOL_CMD_SHUTTER_CLOSE,                     // Close the shutter
        PROTOCOL_CMD_SHUTTER_STOP,                      // Stop the shutter
        PROTOCOL_CMD_SHUTTER_GET_CURRENT_STATE,         // Open, close, opening, closing
        PROTOCOL_CMD_SHUTTER_ANS_CURRENT_STATE,         // Open, close, opening, closing
        PROTOCOL_CMD_SHUTTER_GET_CURRENT_POSITION,       // Return the position register (SW Endstop)
        PROTOCOL_CMD_SHUTTER_ANS_CURRENT_POSITION,       // Return the position register (SW Endstop)
        PROTOCOL_CMD_SHUTTER_SET_ENDSTOP,
        PROTOCOL_CMD_SHUTTER_INCREMENT_ENDSTOP,
        PROTOCOL_CMD_SHUTTER_DECREMENT_ENDSTOP,
        PROTOCOL_CMD_SHUTTER_SAVE_ENDSTOP,
        PROTOCOL_CMD_SHUTTER_SET_VAR,
        PROTOCOL_CMD_SHUTTER_GET_VAR,


        PROTOCOL_COM_NOTHING
    };

    enum ShutterVars
    {
        SHUTTER_VAR_KP,
        SHUTTER_VAR_KI,
        SHUTTER_VAR_KD,

        SHUTTER_VAR_SW_ENDSTOP_CORRECTION,
        SHUTTER_VAR_MY_ADDR,
        SHUTTER_VAR_MAX_ASSORBED_CURRENT,
        SHUTTER_VAR_CUTOFF_CURRENT_DIFFERENCE,
        SHUTTER_VAR_CUTOFF_AVARAGE_WEIGHT,
        SHUTTER_VAR_CURRENT_AVARAGE_WEIGHT,
        SHUTTER_VAR_TURN_ON_TIME
    };

    enum ShutterState
    {
        SHUTTER_STATE_OPENED,
        SHUTTER_STATE_CLOSED,
        SHUTTER_STATE_OPENING,
        SHUTTER_STATE_CLOSING
    };

    class BusDataPacket
    {
        public DeviceCategories SenderType;
        public byte SenderAddress;
        public DeviceCategories DestinationType;
        public byte DestinationAddress;
        public List<byte> Payload = new List<byte>();

        public BusDataPacket()
        {
        }

        public BusDataPacket(DeviceCategories srcType, byte srcAddress, DeviceCategories destType, byte destAddress)
        {
            SenderType = srcType;
            SenderAddress = srcAddress;
            DestinationType = destType;
            DestinationAddress = destAddress;
        }


        /// <summary>
        /// Defines the Low Nibble and the High Nibble for each command. High Nibble is 0x20, Low Nibble is 0x30. 
        /// Command to shutter engines is packed the same way:
        /// 
        /// SOT - SRC_CATEGORY - SRC_ADDR - DST_CATEGORY - DST_ADDR - PAYLOAD - EOT
        /// 
        /// Every section is partioned in Low and High nibble except from the SOT (0x01) & EOT (0x04).
        /// This function creates the package in the right way.
        /// </summary>
        public byte[] GetSerialPacket()
        {
            List<byte> bl = new List<byte>();
            byte highNibble, lowNibble;

            bl.Add((byte)ProtocolSpecialChars.PROT_SPEC_CHAR_START_CHAR);
            highNibble = (byte)(((int)SenderType >> 4) & 0x0f);
            highNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_MSN;
            lowNibble = (byte)(((int)SenderType) & 0x0f);
            lowNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_LSN;
            bl.Add(highNibble);
            bl.Add(lowNibble);

            highNibble = (byte)(((int)SenderAddress >> 4) & 0x0f);
            highNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_MSN;
            lowNibble = (byte)(((int)SenderAddress) & 0x0f);
            lowNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_LSN;
            bl.Add(highNibble);
            bl.Add(lowNibble);

            highNibble = (byte)(((int)DestinationType >> 4) & 0x0f);
            highNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_MSN;
            lowNibble = (byte)(((int)DestinationType) & 0x0f);
            lowNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_LSN;
            bl.Add(highNibble);
            bl.Add(lowNibble);

            highNibble = (byte)(((int)DestinationAddress >> 4) & 0x0f);
            highNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_MSN;
            lowNibble = (byte)(((int)DestinationAddress) & 0x0f);
            lowNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_LSN;
            bl.Add(highNibble);
            bl.Add(lowNibble);

            foreach (byte b in Payload)
            {
                highNibble = (byte)(((int)b >> 4) & 0x0f);
                highNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_MSN;
                lowNibble = (byte)(((int)b) & 0x0f);
                lowNibble |= (byte)ProtocolSpecialChars.PROT_SPEC_CHAR_LSN;
                bl.Add(highNibble);
                bl.Add(lowNibble);
            }

            bl.Add((byte)ProtocolSpecialChars.PROT_SPEC_CHAR_END_OF_TRANSMISSION);
            return bl.ToArray();
        }
    };

    class PacketReceiverEventArgs : EventArgs
    {
        public BusDataPacket Packet;
        public PacketReceiverEventArgs(BusDataPacket p)
        {
            Packet = p;
        }
    };

    class BusInterface
    {
        public static ASCOM.Utilities.Serial asport;
        BusDataPacket txPacket;


        public BusInterface()
        {

            asport = new ASCOM.Utilities.Serial();
            
        }

        List<int> ReceiveBuffer = new List<int>();

        public delegate void PacketReceivedEventHandler(object sender, EventArgs e);
        public event PacketReceivedEventHandler PacketReceivedEvent;

        public delegate void AckReceivedHandler(object sender, EventArgs e);
        public event AckReceivedHandler AckReceivedEvent;

        public delegate void NAckReceivedHandler(object sender, EventArgs e);
        public event NAckReceivedHandler NAckReceivedEvent;

        void asport_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int _c;                     // Port von SerialPort zu ASCOM Port wechseln
            //int c;

            byte[] getlength = txPacket.GetSerialPacket();
            int length = getlength.Length;
            byte[] received_data = Switch.asport.ReceiveCountedBinary(length);

            int d = received_data.Length;

            byte t = 0;
            Switch.asport.ReceiveTimeoutMs = 50;

            //port.ReadTimeout = 50;
            while (d > 0) 
            {
                _c = Switch.asport.ReceiveByte();
                //c = port.ReadByte();

                //Console.WriteLine(c.ToString());
                if (_c == (int)ProtocolSpecialChars.PROT_SPEC_CHAR_START_CHAR)
                {
                    ReceiveBuffer.Clear();
                    //ReceiveBuffer.Add(_c);
                }
                else if (_c == (int)ProtocolSpecialChars.PROT_SPEC_CHAR_END_OF_TRANSMISSION)
                {
                    //ReceiveBuffer.Add(_c);
                    ParseBuffer();
                }
                else if (_c == (int)ProtocolSpecialChars.PROT_SPEC_CHAR_ACK)
                {
                    if (AckReceivedEvent != null)
                        AckReceivedEvent(this, null);
                }
                else if (_c == (int)ProtocolSpecialChars.PROT_SPEC_CHAR_NACK)
                {
                    if (NAckReceivedEvent != null)
                        NAckReceivedEvent(this, null);
                }
                else
                {
                    if ((_c & 0xf0) == 0x20)
                    {
                        t = (byte)(_c << 4);
                    }
                    else if ((_c & 0xf0) == 0x30)
                    {
                        t |= (byte)(_c & 0x0f);
                        ReceiveBuffer.Add(t);
                    }
                }
            }


        }

        private void ParseBuffer()
        {
            BusDataPacket packet = new BusDataPacket();
            byte t = 0;
            byte currentByte = 0;
            for (int i = 0; i < ReceiveBuffer.Count; i++)
            {
                currentByte = (byte)ReceiveBuffer[i];
                switch (i)
                {
                    case 0:
                        packet.SenderType = (DeviceCategories)ReceiveBuffer[i];
                        break;
                    case 1:
                        packet.SenderAddress = (byte)ReceiveBuffer[i];
                        break;
                    case 2:
                        packet.DestinationType = (DeviceCategories)ReceiveBuffer[i];
                        break;
                    case 3:
                        packet.DestinationAddress = (byte)ReceiveBuffer[i];
                        break;
                    default:
                        packet.Payload.Add(currentByte);
                        break;
                }
            }
            if (PacketReceivedEvent != null)
            {
                PacketReceivedEvent(this, new PacketReceiverEventArgs(packet));
            }
        }

        public string[] GetPorts()
        {
            return SerialPort.GetPortNames();
        }

        public static void Connect(string portname)
        {

            if (Switch.asport.Connected == true)
                Switch.asport.Connected = false;
            Switch.asport.PortName = portname;
            Switch.asport.Handshake = SerialHandshake.None;
            Switch.asport.Speed = (SerialSpeed)57600;
            Switch.asport.DTREnable = false;
            Switch.asport.RTSEnable = false;
            Switch.asport.StopBits = SerialStopBits.One;
            Switch.asport.Parity = SerialParity.None;
            Switch.asport.DataBits = 8;

            Switch.asport.Connected = true;

        }



        public static void Disconnect()
        {
            Switch.asport.Connected = false;
        }

        public void CreateHeader(DeviceCategories srcDeviceCategory, byte srcAddress, DeviceCategories destDeviceCategory, byte destAddress)
        {
            txPacket = new BusDataPacket(srcDeviceCategory, srcAddress, destDeviceCategory, destAddress);
        }

        public void AppendPayload(byte data)
        {
            if (txPacket != null)
                txPacket.Payload.Add(data);
        }

        public void SendPacket()
        {
            byte[] txBuffer = txPacket.GetSerialPacket();
            Switch.asport.DTREnable = false;                //Data Terminal Ready Signal wird unterbunden
            Switch.asport.TransmitBinary(txBuffer);
            Thread.Sleep(100);
            Switch.asport.DTREnable = true;
        }
    }
}
