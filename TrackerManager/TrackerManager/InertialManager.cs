using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace TrackerManager
{
    internal class InertialManager
    {
        private List<InertialTracker> devices = new List<InertialTracker>();

        public InertialManager(List<string> inertialPorts)
        {
            foreach (var item in inertialPorts)
            {
                var device = new InertialTracker(item);
                device.Id = inertialPorts.IndexOf(item);
                devices.Add(device);
            }
        }

        public List<InertialTracker> Devices => devices;

        public void Dispose()
        {
            foreach (var item in devices)
            {
                item.Dispose();
            }
        }
    }

    public enum Type : int
    {
        Accelerometer = 1,
        Gyroscope = 2
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Message
    {
        public int header;
        public int type;
        public Vector3 value;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct InertialEvent : IEvent
    {
        public InertialTracker device;
        public Type type;
        public Vector3 data;
        public double time;

        public override string ToString()
        {
            return $"{device.Id}, {(int)type}, {data.X}, {data.Y}, {data.Z}, {time}";
        }

        public void ToFloats(BinaryWriter writer)
        {
            writer.Write((float)type);
            writer.Write((float)device.hardwareId);
            writer.Write((float)time);
            writer.Write(data.X);
            writer.Write(data.Y);
            writer.Write(data.Z);
        }
    }

    public enum Commands : int
    {
        SendId = 1,
        Start = 2,
        Identify = 3,
        Unidentify = 4
    }

    public class InertialTracker
    {
        private SerialPort serialPort;
        private Thread thread;

        public delegate void UpdateEventCallback(InertialEvent f);
        public event UpdateEventCallback OnEvent;

        /// <summary>
        /// User assignable id for quick indexing and tables.
        /// </summary>
        public int Id;

        /// <summary>
        /// Static id baked into the hardware
        /// </summary>
        public byte[] hardwareIdBytes;
        public byte hardwareId;

        private bool mightHaveCommand;
        private ConcurrentQueue<Commands> commands;

        private bool initialised;

        public void SendCommand(Commands command)
        {
            commands.Enqueue(command);
            mightHaveCommand = true;
        }

        public InertialTracker(string portId)
        {
            commands = new ConcurrentQueue<Commands>();

            serialPort = new SerialPort(portId);

            serialPort.PortName = $"{portId.ToUpper()}";
            serialPort.BaudRate = 115200;
            serialPort.Parity = Parity.None;
            serialPort.DataBits = 8;
            serialPort.StopBits = StopBits.One;
            serialPort.ReadTimeout = 1000;
            serialPort.RtsEnable = true; // Necessary for Arduino even though Serial does not configure it.
            serialPort.DtrEnable = true;

            serialPort.Open();

            initialised = false;
            hardwareIdBytes = new byte[12];

            thread = new Thread(new ThreadStart(ReadWorker));
            thread.Start();
        }

        private void ReadWorker()
        {
            var buffer = new byte[256];
            var position = 0;
            var length = buffer.Length;
            var ptr = GCHandle.Alloc(buffer, GCHandleType.Pinned);
            var messageLength = Marshal.SizeOf<Message>();
            var sendBuffer = new byte[1];

            Program.Log("Connected Inertial Device " + serialPort.PortName);

            try
            {
                while (true)
                {
                    var remaining = length - position;
                    var read = serialPort.Read(buffer, position, remaining);
                    position += read;
                    if (position >= messageLength)
                    {
                        // If we potentially have enough data in the buffer to
                        // make a frame, try and process the buffer

                        // This section advances byte by byte to match the frame
                        // header; if found it reads the whole frame and
                        // advances by that amount.
                        // This process repeats until all the available data has
                        // been processed.

                        var i = 0;
                        while (i < (position - messageLength))
                        {
                            if (buffer[i] == 0xFF && buffer[i + 1] == 0xFD && buffer[i + 2] == 0xFF && buffer[i + 3] == 0xFF) // Little Endian - least significant byte at smallest address/lowest index
                            {
                                // Found a frame
                                var messagePtr = ptr.AddrOfPinnedObject() + i;
                                var message = (Message)Marshal.PtrToStructure(messagePtr, typeof(Message));
                                i += messageLength;

                                var ev = new InertialEvent();
                                ev.device = this;
                                ev.time = Program.Time;

                                switch (message.type & 0xFF)
                                {
                                    case 1:
                                        ev.type = Type.Accelerometer;
                                        ev.data = message.value;
                                        if (initialised)
                                        {
                                            OnEvent?.Invoke(ev);
                                        }
                                        break;
                                    case 2:
                                        ev.type = Type.Gyroscope;
                                        ev.data = message.value;
                                        if (initialised)
                                        {
                                            OnEvent?.Invoke(ev);
                                        }
                                        break;
                                    case 3:
                                        Marshal.Copy(messagePtr + 8, hardwareIdBytes, 0, 12);
                                        hardwareId = hardwareIdBytes[0];
                                        initialised = true;
                                        break;
                                }
                            }
                            else
                            {
                                i++;
                            }
                        }

                        // We have tested up to i now, we can dump anything 
                        // before it to make space for the next read.
                        // The small bit after i but before position may hold
                        // the beginning of the next frame, so move that to 
                        // the start to keep it around.

                        Buffer.BlockCopy(buffer, i, buffer, 0, position - i);
                        position = position - i;
                    }

                    if (!initialised)
                    {
                        SendCommand(Commands.SendId);
                    }

                    if (mightHaveCommand)
                    {
                        while (commands.TryDequeue(out var command))
                        {
                            sendBuffer[0] = (byte)command;
                            serialPort.Write(sendBuffer, 0, 1);
                        }
                        mightHaveCommand = false;
                    }
                }
            }
            finally
            {
                ptr.Free();
            }
        }

        public void Dispose()
        {
            serialPort.Close();
            serialPort.Dispose();
            thread.Join();
        }
    }
}
