//tabs=4
// --------------------------------------------------------------------------------
// TODO fill in this information for your driver, then remove this line!
//
// ASCOM Switch driver for Shutter
//
// Description:	Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam 
//				nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam 
//				erat, sed diam voluptua. At vero eos et accusam et justo duo 
//				dolores et ea rebum. Stet clita kasd gubergren, no sea takimata 
//				sanctus est Lorem ipsum dolor sit amet.
//
// Implements:	ASCOM Switch interface version: <To be completed by driver developer>
// Author:		(XXX) Your N. Here <your@email.here>
//
// Edit Log:
//
// Date			Who	Vers	Description
// -----------	---	-----	-------------------------------------------------------
// dd-mmm-yyyy	XXX	6.0.0	Initial edit, created from ASCOM driver template
// --------------------------------------------------------------------------------
//


// This is used to define code in the template that is specific to one class implementation
// unused code canbe deleted and this definition removed.
#define Switch

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Runtime.InteropServices;

using ASCOM;
using ASCOM.Astrometry;
using ASCOM.Astrometry.AstroUtils;
using ASCOM.Utilities;
using ASCOM.DeviceInterface;
using System.Globalization;
using System.Collections;
using System.Linq;
using System.Threading.Tasks;
using System.IO;
using System.IO.Ports;
using System.Threading;

namespace ASCOM.Shutter
{
    //
    // Your driver's DeviceID is ASCOM.Shutter.Switch
    //
    // The Guid attribute sets the CLSID for ASCOM.Shutter.Switch
    // The ClassInterface/None addribute prevents an empty interface called
    // _Shutter from being created and used as the [default] interface
    //
    // TODO Replace the not implemented exceptions with code to implement the function or
    // throw the appropriate ASCOM exception.
    //

    /// <summary>
    /// ASCOM Switch Driver for Shutter.
    /// </summary>
    [Guid("f24a7255-6ab3-416d-a6e2-670b50182df0")]
    [ClassInterface(ClassInterfaceType.None)]
    public class Switch : ISwitchV2                                    //used to be public
    {
        /// <summary>
        /// ASCOM DeviceID (COM ProgID) for this driver.
        /// The DeviceID is used by ASCOM applications to load the driver at runtime.
        /// </summary>
        internal static string driverID = "ASCOM.Shutter.Switch";
        // TODO Change the descriptive string for your driver then remove this line
        /// <summary>
        /// Driver description that displays in the ASCOM Chooser.
        /// </summary>
        private static string driverDescription = "ASCOM Switch Driver for Shutter";
        
        internal static string comPortProfileName = "COM Port"; // Constants used for Profile persistence
        internal static string comPortDefault = "COM1";
        internal static string traceStateProfileName = "Trace Level";
        internal static string traceStateDefault = "false";

        internal static String comPort; // Variables to hold the currrent device configuration

        public static ASCOM.Utilities.Serial asport; //ASCOM Port used in Businterface functions
        
        internal static BusInterface bus;

        /// <summary>
        /// Private variable to hold the connected state
        /// </summary>
        private bool connectedState;

        /// <summary>
        /// Private variable to hold an ASCOM Utilities object
        /// </summary>
        private Util utilities;

        /// <summary>
        /// Private variable to hold an ASCOM AstroUtilities object to provide the Range method
        /// </summary>
        private AstroUtils astroUtilities;

        /// <summary>
        /// Variable to hold the trace logger object (creates a diagnostic log file with information that you specify)
        /// </summary>
        internal static TraceLogger tl;

        /// <summary>
        /// Initializes a new instance of the <see cref="Shutter"/> class.
        /// Must be public for COM registration.
        /// </summary>
        public Switch()
        {
            tl = new TraceLogger("", "Shutter");
            ReadProfile(); // Read device configuration from the ASCOM Profile store

            tl.LogMessage("Switch", "Starting initialisation");

            connectedState = false; // Initialise connected to false
            utilities = new Util(); //Initialise util object
            astroUtilities = new AstroUtils(); // Initialise astro utilities object
            //TODO: Implement your additional construction here

            tl.LogMessage("Switch", "Completed initialisation");

            /// Byte packet that gets sent to bus. 
            //bus = new BusInterface();
            //bus.PacketReceivedEvent += bus_PacketReceivedEvent;


        }


        //
        // PUBLIC COM INTERFACE ISwitchV2 IMPLEMENTATION
        //

        #region Common properties and methods.

        /// <summary>
        /// Displays the Setup Dialog form.
        /// If the user clicks the OK button to dismiss the form, then
        /// the new settings are saved, otherwise the old values are reloaded.
        /// THIS IS THE ONLY PLACE WHERE SHOWING USER INTERFACE IS ALLOWED!
        /// </summary>
        public void SetupDialog()
        {
            // consider only showing the setup dialog if not connected
            // or call a different dialog if connected
            if (IsConnected)
                System.Windows.Forms.MessageBox.Show("Already connected, just press OK");
            
            using (SetupDialogForm F = new SetupDialogForm())
            {
                var result = F.ShowDialog();
                if (result == System.Windows.Forms.DialogResult.OK)
                {
                    WriteProfile(); // Persist device configuration values to the ASCOM Profile store
                }
            }
        }

        public ArrayList SupportedActions
        {
            get
            {
                tl.LogMessage("SupportedActions Get", "Returning empty arraylist");
                return new ArrayList();
            }
        }

        public string Action(string actionName, string actionParameters)
        {
            LogMessage("", "Action {0}, parameters {1} not implemented", actionName, actionParameters);
            throw new ASCOM.ActionNotImplementedException("Action " + actionName + " is not implemented by this driver");
        }

        public void CommandBlind(string command, bool raw)
        {
            CheckConnected("CommandBlind");
            // Call CommandString and return as soon as it finishes
            this.CommandString(command, raw);
            // or
            throw new ASCOM.MethodNotImplementedException("CommandBlind");
            // DO NOT have both these sections!  One or the other
        }

        public bool CommandBool(string command, bool raw)
        {
            CheckConnected("CommandBool");
            string ret = CommandString(command, raw);
            // TODO decode the return string and return true or false
            // or
            throw new ASCOM.MethodNotImplementedException("CommandBool");
            // DO NOT have both these sections!  One or the other
        }

        public string CommandString(string command, bool raw)
        {
            CheckConnected("CommandString");
            // it's a good idea to put all the low level communication with the device here,
            // then all communication calls this function
            // you need something to ensure that only one command is in progress at a time

            throw new ASCOM.MethodNotImplementedException("CommandString");
        }

        public void Dispose()
        {
            // Clean up the tracelogger and util objects
            tl.Enabled = false;
            tl.Dispose();
            tl = null;
            utilities.Dispose();
            utilities = null;
            astroUtilities.Dispose();
            astroUtilities = null;
        }

        public bool Connected
        {
            get
            {
                LogMessage("Connected", "Get {0}", IsConnected);
                return IsConnected;
            }
            set
            {
                tl.LogMessage("Connected", "Set {0}", value);
                if (value == IsConnected)
                    return;

                if (value)
                {
                    asport = new ASCOM.Utilities.Serial();
                    connectedState = true;
                    LogMessage("Connected Set", "Connecting to port {0}", comPort);
                    // TODO connect to the device

                    BusInterface.Connect(comPort);  
                }
                else
                {
                    connectedState = false;
                    LogMessage("Connected Set", "Disconnecting from port {0}", comPort);
                    // TODO disconnect from the device
                    //asport.Connected = false;
                    BusInterface.Disconnect();
                }
            }
        }

        //public void TestConnection()
        //{
        //    if (asport.Connected == true)
        //    {
        //        ExecuteShutterCommand(1, ProtocolCommands.PROTOCOL_CMD_SHUTTER_OPEN);
        //        ExecuteShutterCommand(2, ProtocolCommands.PROTOCOL_CMD_SHUTTER_OPEN);
        //        Thread.Sleep(4000);
        //        ExecuteShutterCommand(3, ProtocolCommands.PROTOCOL_CMD_SHUTTER_OPEN);
        //        ExecuteShutterCommand(4, ProtocolCommands.PROTOCOL_CMD_SHUTTER_OPEN);
        //    }

        //    else return;
        //}

        public string Description
        {
            // TODO customise this device description
            get
            {
                tl.LogMessage("Description Get", driverDescription);
                return driverDescription;
            }
        }

        public string DriverInfo
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                // TODO customise this driver description
                string driverInfo = "Information about the driver itself. Version: " + String.Format(CultureInfo.InvariantCulture, "{0}.{1}", version.Major, version.Minor);
                tl.LogMessage("DriverInfo Get", driverInfo);
                return driverInfo;
            }
        }

        public string DriverVersion
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverVersion = String.Format(CultureInfo.InvariantCulture, "{0}.{1}", version.Major, version.Minor);
                tl.LogMessage("DriverVersion Get", driverVersion);
                return driverVersion;
            }
        }

        public short InterfaceVersion
        {
            // set by the driver wizard
            get
            {
                LogMessage("InterfaceVersion Get", "2");
                return Convert.ToInt16("2");
            }
        }

        public string Name
        {
            get
            {
                string name = "ASCOM Shutter Control";
                tl.LogMessage("Name Get", name);
                return name;
            }
        }

        #endregion

        #region ISwitchV2 Implementation

        private short numSwitch = 2;

        /// <summary>
        /// The number of switches managed by this driver
        /// </summary>
        public short MaxSwitch
        {
            get
            {
                tl.LogMessage("MaxSwitch Get", numSwitch.ToString());
                return this.numSwitch;
            }
        }
        public string _name;
        /// <summary>
        /// Return the name of switch n
        /// </summary>
        /// <param name="id">The switch number to return</param>
        /// <returns>
        /// The name of the switch
        /// </returns>
        public string GetSwitchName(short id)
        {
            Validate("GetSwitchName", id);
            //tl.LogMessage("GetSwitchName", string.Format("GetSwitchName({0}) - not implemented", id));
            //throw new MethodNotImplementedException("GetSwitchName");
            // or
            
            switch (id)
            {
                case 0:
                    _name = "Open";
                    break;
                case 1:
                    _name = "Close";
                    break;
            }
            tl.LogMessage("GetSwitchName", string.Format("GetSwitchName({0}): Switch{0}_{1}", id, _name));
            return _name;
        }

        /// <summary>
        /// Sets a switch name to a specified value
        /// </summary>
        /// <param name="id">The number of the switch whose name is to be set</param>
        /// <param name="name">The name of the switch</param>
        public void SetSwitchName(short id, string name)
        {
            Validate("SetSwitchName", id);
            tl.LogMessage("SetSwitchName", string.Format("SetSwitchName({0}) = {1}", id, name));
            //throw new MethodNotImplementedException("SetSwitchName");
        }


        /// <summary>
        /// Gets the switch description.
        /// </summary>
        /// <param name="id">The id.</param>
        /// <returns></returns>
        public string description;

        public string GetSwitchDescription(short id)
        {
            Validate("GetSwitchDescription", id);


            if (id == 0)
            {
                description = "Opening all Shutters.";
            }

            else if (id == 1)
            {
                description = "Closing all Shutters.";
            }

            tl.LogMessage("GetSwitchDescription", string.Format("GetSwitchDescription({0}): {1}", id, description));

            //throw new MethodNotImplementedException("GetSwitchDescription");

            return description;
        }

        /// <summary>
        /// Reports if the specified switch can be written to.
        /// This is false if the switch cannot be written to, for example a limit switch or a sensor.
        /// The default is true.
        /// </summary>
        /// <param name="id">The number of the switch whose write state is to be returned</param><returns>
        ///   <c>true</c> if the switch can be written to, otherwise <c>false</c>.
        /// </returns>
        /// <exception cref="MethodNotImplementedException">If the method is not implemented</exception>
        /// <exception cref="InvalidValueException">If id is outside the range 0 to MaxSwitch - 1</exception>
        public bool CanWrite(short id)
        {
            Validate("CanWrite", id);
            // default behavour is to report true
            tl.LogMessage("CanWrite", string.Format("CanWrite({0}) - default true", id));
            return true;
            // implementation should report the correct behaviour
            //tl.LogMessage("CanWrite", string.Format("CanWrite({0}) - not implemented", id));
            //throw new MethodNotImplementedException("CanWrite");
        }

        #region boolean switch members

        /// <summary>
        /// Return the state of switch n
        /// a multi-value switch must throw a not implemented exception
        /// </summary>
        /// <param name="id">The switch number to return</param>
        /// <returns>
        /// True or false
        /// </returns>
        public bool GetSwitch(short id)
        {
            Validate("GetSwitch", id);
            tl.LogMessage("GetSwitch", string.Format("GetSwitch({0}) - not implemented", id));
            throw new MethodNotImplementedException("GetSwitch");
        }

        /// <summary>
        /// Sets a switch to the specified state
        /// If the switch cannot be set then throws a MethodNotImplementedException.
        /// A multi-value switch must throw a not implemented exception
        /// setting it to false will set it to its minimum value.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="state"></param>
        public void SetSwitch(short id, bool state)
        {

            Switch.bus = new BusInterface();
            Validate("SetSwitch", id);

            if (!CanWrite(id))
            {
                var str = string.Format("SetSwitch({0}) - Cannot Write", id);
                tl.LogMessage("SetSwitch", str);
                throw new MethodNotImplementedException(str);
            }

            if (state == true)
            {
                if (id == 0)
                {
                    //Commands to Open/Close the shutter
                    OpenShutter();
                    SetSwitch(1, false);
                }

                else if (id == 1)
                {
                    CloseShutter();
                    SetSwitch(0, false);
                }
            }
            
            //bus.PacketReceivedEvent += bus_PacketReceivedEvent;

            
            tl.LogMessage("SetSwitch", string.Format("SetSwitch({0}) = {1}", id, state));
            //throw new MethodNotImplementedException("SetSwitch");
        }

        #endregion

        #region Specified Functions

        internal static void ExecuteShutterCommand(byte shutterAddr, ProtocolCommands cmd)
        {
            Switch.bus = new BusInterface();
            bus.CreateHeader(DeviceCategories.DEV_CAT_MASTER, 0, DeviceCategories.DEV_CAT_SHUTTER, shutterAddr);
            bus.AppendPayload((byte)cmd);
            bus.SendPacket();
        }

        public static void OpenShutter()
        {
            ExecuteShutterCommand(1, ProtocolCommands.PROTOCOL_CMD_SHUTTER_OPEN);
            ExecuteShutterCommand(2, ProtocolCommands.PROTOCOL_CMD_SHUTTER_OPEN);
            Thread.Sleep(15000);
            ExecuteShutterCommand(3, ProtocolCommands.PROTOCOL_CMD_SHUTTER_OPEN);
            ExecuteShutterCommand(4, ProtocolCommands.PROTOCOL_CMD_SHUTTER_OPEN);
        }

        public static void CloseShutter()
        {
            ExecuteShutterCommand(4, ProtocolCommands.PROTOCOL_CMD_SHUTTER_CLOSE);
            ExecuteShutterCommand(3, ProtocolCommands.PROTOCOL_CMD_SHUTTER_CLOSE);
            Thread.Sleep(15000);
            ExecuteShutterCommand(2, ProtocolCommands.PROTOCOL_CMD_SHUTTER_CLOSE);
            ExecuteShutterCommand(1, ProtocolCommands.PROTOCOL_CMD_SHUTTER_CLOSE);
        }

        public static void bus_PacketReceivedEvent(object sender, EventArgs e)
        {
            PacketReceiverEventArgs EventArgs = (PacketReceiverEventArgs)e;
            BusDataPacket p = EventArgs.Packet;

            ExecuteShutterCommand(1, ProtocolCommands.PROTOCOL_CMD_SHUTTER_GET_CURRENT_STATE);
            ExecuteShutterCommand(2, ProtocolCommands.PROTOCOL_CMD_SHUTTER_GET_CURRENT_STATE);
            ExecuteShutterCommand(3, ProtocolCommands.PROTOCOL_CMD_SHUTTER_GET_CURRENT_STATE);
            ExecuteShutterCommand(4, ProtocolCommands.PROTOCOL_CMD_SHUTTER_GET_CURRENT_STATE);

            if(p.Payload.Count != 0)
            {
                switch ((ProtocolCommands)p.Payload[0])
                {
                    case ProtocolCommands.PROTOCOL_CMD_SHUTTER_ANS_CURRENT_STATE:

                        string state = ((ShutterState)p.Payload[1]).ToString().Substring(14);

                        tl.LogMessage("SetSwitch", string.Format("Shutter {0}: {1}", p.SenderAddress.ToString(), state));
                        //throw new MethodNotImplementedException("SetSwitch");
                        throw new MethodAccessException("SetSwitch");
                }
            }
        }

        #endregion

        #region analogue members

        /// <summary>
        /// returns the maximum value for this switch
        /// boolean switches must return 1.0
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public double MaxSwitchValue(short id)
        {
            Validate("MaxSwitchValue", id);
            // boolean switch implementation:
            return 1;
            // or
            //tl.LogMessage("MaxSwitchValue", string.Format("MaxSwitchValue({0}) - not implemented", id));
            //throw new MethodNotImplementedException("MaxSwitchValue");
        }

        /// <summary>
        /// returns the minimum value for this switch
        /// boolean switches must return 0.0
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public double MinSwitchValue(short id)
        {
            Validate("MinSwitchValue", id);
            // boolean switch implementation:
            return 0;
            // or
            //tl.LogMessage("MinSwitchValue", string.Format("MinSwitchValue({0}) - not implemented", id));
            //throw new MethodNotImplementedException("MinSwitchValue");
        }

        /// <summary>
        /// returns the step size that this switch supports. This gives the difference between
        /// successive values of the switch.
        /// The number of values is ((MaxSwitchValue - MinSwitchValue) / SwitchStep) + 1
        /// boolean switches must return 1.0, giving two states.
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public double SwitchStep(short id)
        {
            Validate("SwitchStep", id);
            // boolean switch implementation:
            return 1;
            // or
            //tl.LogMessage("SwitchStep", string.Format("SwitchStep({0}) - not implemented", id));
            //throw new MethodNotImplementedException("SwitchStep");
        }

        /// <summary>
        /// returns the analogue switch value for switch id
        /// boolean switches must throw a not implemented exception
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public double GetSwitchValue(short id)
        {
            Validate("GetSwitchValue", id);
            tl.LogMessage("GetSwitchValue", string.Format("GetSwitchValue({0}) - not implemented", id));
            throw new MethodNotImplementedException("GetSwitchValue");
        }

        /// <summary>
        /// set the analogue value for this switch.
        /// If the switch cannot be set then throws a MethodNotImplementedException.
        /// If the value is not between the maximum and minimum then throws an InvalidValueException
        /// boolean switches must throw a not implemented exception.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="value"></param>
        public void SetSwitchValue(short id, double value)
        {
            Validate("SetSwitchValue", id, value);
            if (!CanWrite(id))
            {
                tl.LogMessage("SetSwitchValue", string.Format("SetSwitchValue({0}) - Cannot write", id));
                throw new ASCOM.MethodNotImplementedException(string.Format("SetSwitchValue({0}) - Cannot write", id));
            }
            tl.LogMessage("SetSwitchValue", string.Format("SetSwitchValue({0}) = {1} - not implemented", id, value));

            //throw new MethodNotImplementedException("SetSwitchValue");

        }

        #endregion

        #endregion

        #region private methods

        /// <summary>
        /// Checks that the switch id is in range and throws an InvalidValueException if it isn't
        /// </summary>
        /// <param name="message">The message.</param>
        /// <param name="id">The id.</param>
        private void Validate(string message, short id)
        {
            if (id < 0 || id >= numSwitch)
            {
                tl.LogMessage(message, string.Format("Switch {0} not available, range is 0 to {1}", id, numSwitch - 1));
                throw new InvalidValueException(message, id.ToString(), string.Format("0 to {0}", numSwitch - 1));
            }
        }

        /// <summary>
        /// Checks that the switch id and value are in range and throws an
        /// InvalidValueException if they are not.
        /// </summary>
        /// <param name="message">The message.</param>
        /// <param name="id">The id.</param>
        /// <param name="value">The value.</param>
        private void Validate(string message, short id, double value)
        {
            Validate(message, id);
            var min = MinSwitchValue(id);
            var max = MaxSwitchValue(id);
            if (value < min || value > max)
            {
                tl.LogMessage(message, string.Format("Value {1} for Switch {0} is out of the allowed range {2} to {3}", id, value, min, max));
                throw new InvalidValueException(message, value.ToString(), string.Format("Switch({0}) range {1} to {2}", id, min, max));
            }
        }

        /// <summary>
        /// Checks that the number of states for the switch is correct and throws a methodNotImplemented exception if not.
        /// Boolean switches must have 2 states and multi-value switches more than 2.
        /// </summary>
        /// <param name="message"></param>
        /// <param name="id"></param>
        /// <param name="expectBoolean"></param>
        //private void Validate(string message, short id, bool expectBoolean)
        //{
        //    Validate(message, id);
        //    var ns = (int)(((MaxSwitchValue(id) - MinSwitchValue(id)) / SwitchStep(id)) + 1);
        //    if ((expectBoolean && ns != 2) || (!expectBoolean && ns <= 2))
        //    {
        //        tl.LogMessage(message, string.Format("Switch {0} has the wriong number of states", id, ns));
        //        throw new MethodNotImplementedException(string.Format("{0}({1})", message, id));
        //    }
        //}

        #endregion

        #region Private properties and methods
        // here are some useful properties and methods that can be used as required
        // to help with driver development

        #region ASCOM Registration

        // Register or unregister driver for ASCOM. This is harmless if already
        // registered or unregistered. 
        //
        /// <summary>
        /// Register or unregister the driver with the ASCOM Platform.
        /// This is harmless if the driver is already registered/unregistered.
        /// </summary>
        /// <param name="bRegister">If <c>true</c>, registers the driver, otherwise unregisters it.</param>
        private static void RegUnregASCOM(bool bRegister)
        {
            using (var P = new ASCOM.Utilities.Profile())
            {
                P.DeviceType = "Switch";
                if (bRegister)
                {
                    P.Register(driverID, driverDescription);
                }
                else
                {
                    P.Unregister(driverID);
                }
            }
        }

        /// <summary>
        /// This function registers the driver with the ASCOM Chooser and
        /// is called automatically whenever this class is registered for COM Interop.
        /// </summary>
        /// <param name="t">Type of the class being registered, not used.</param>
        /// <remarks>
        /// This method typically runs in two distinct situations:
        /// <list type="numbered">
        /// <item>
        /// In Visual Studio, when the project is successfully built.
        /// For this to work correctly, the option <c>Register for COM Interop</c>
        /// must be enabled in the project settings.
        /// </item>
        /// <item>During setup, when the installer registers the assembly for COM Interop.</item>
        /// </list>
        /// This technique should mean that it is never necessary to manually register a driver with ASCOM.
        /// </remarks>
        [ComRegisterFunction]
        public static void RegisterASCOM(Type t)
        {
            RegUnregASCOM(true);
        }

        /// <summary>
        /// This function unregisters the driver from the ASCOM Chooser and
        /// is called automatically whenever this class is unregistered from COM Interop.
        /// </summary>
        /// <param name="t">Type of the class being registered, not used.</param>
        /// <remarks>
        /// This method typically runs in two distinct situations:
        /// <list type="numbered">
        /// <item>
        /// In Visual Studio, when the project is cleaned or prior to rebuilding.
        /// For this to work correctly, the option <c>Register for COM Interop</c>
        /// must be enabled in the project settings.
        /// </item>
        /// <item>During uninstall, when the installer unregisters the assembly from COM Interop.</item>
        /// </list>
        /// This technique should mean that it is never necessary to manually unregister a driver from ASCOM.
        /// </remarks>
        [ComUnregisterFunction]
        public static void UnregisterASCOM(Type t)
        {
            RegUnregASCOM(false);
        }

        #endregion

        /// <summary>
        /// Returns true if there is a valid connection to the driver hardware
        /// </summary>
        private bool IsConnected
        {
            get
            {
                // TODO check that the driver hardware connection exists and is connected to the hardware
                return connectedState;
            }
        }

        /// <summary>
        /// Use this function to throw an exception if we aren't connected to the hardware
        /// </summary>
        /// <param name="message"></param>
        private void CheckConnected(string message)
        {
            if (!IsConnected)
            {
                throw new ASCOM.NotConnectedException(message);
            }
        }

        /// <summary>
        /// Read the device configuration from the ASCOM Profile store
        /// </summary>
        internal void ReadProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Switch";
                tl.Enabled = Convert.ToBoolean(driverProfile.GetValue(driverID, traceStateProfileName, string.Empty, traceStateDefault));
                comPort = driverProfile.GetValue(driverID, comPortProfileName, string.Empty, comPortDefault);
            }
        }

        /// <summary>
        /// Write the device configuration to the  ASCOM  Profile store
        /// </summary>
        internal void WriteProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Switch";
                driverProfile.WriteValue(driverID, traceStateProfileName, tl.Enabled.ToString());
                driverProfile.WriteValue(driverID, comPortProfileName, comPort.ToString());
            }
        }

        /// <summary>
        /// Log helper function that takes formatted strings and arguments
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        /// <param name="args"></param>
        internal static void LogMessage(string identifier, string message, params object[] args)
        {
            var msg = string.Format(message, args);
            tl.LogMessage(identifier, msg);
        }
        #endregion
    }//end of class

}//end of namespace
