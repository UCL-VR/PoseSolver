using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Net;
using System.Net.Sockets;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Xml.Linq;

namespace TrackerManager
{
    /// <summary>
    /// The TrackerManager service establishes a connection to and recieves data 
    /// from all the IMU devices.
    /// </summary>
    internal class Program
    {
        private string phaseSpaceAddress = "phasespace.cs.ucl.ac.uk";

        public List<string> InertialPorts = new List<string>()
        {
            "COM5"
        };

        private OpticalManager opticalManager = null;
        private InertialManager inertialManager = null;

        private static int SendPort = 24693;

        public void Start()
        {
            inertialManager = new InertialManager(InertialPorts);
            //opticalManager = new OpticalManager(phaseSpaceAddress);
        }

        public void Stop()
        {
            if (inertialManager != null)
            {
                inertialManager.Dispose();
            }
            if (opticalManager != null)
            {
                opticalManager.Dispose();
            }
        }

        public static void Log(string m)
        {
            Messages.Enqueue(m);
        }

        public static ConcurrentQueue<string> Messages = new ConcurrentQueue<string>();

        private ConcurrentQueue<IEvent> MakeEventQueue()
        {
            // This is the queue that will have all events written into it

            ConcurrentQueue<IEvent> events = new ConcurrentQueue<IEvent>();

            // This snippet sets up the inertial manager to write to the stream.

            if (inertialManager != null)
            {
                foreach (var item in inertialManager.Devices)
                {
                    item.OnEvent += (ev) =>
                    {
                        events.Enqueue(ev);
                    };
                }
            }

            // This snippet sets up the optical manager to write to the stream

            if (opticalManager != null)
            {
                opticalManager.OnEvent += (ev) =>
                {
                    events.Enqueue(ev);
                };
            }

            return events;
        }

        /// <summary>
        /// Captures all device events into a stream. The stream will be written
        /// to from a single thread, though not necessarily the callers.
        /// </summary>
        public void CaptureCsv(BinaryWriter writer)
        {
            // This is the shared event queue that demultiplexes all event streams

            var events = MakeEventQueue();

            // This is the worker that will write all events to file asynchronously

            var worker = new Thread(() =>
            {
                while (true)
                {
                    if (events.TryDequeue(out var ev))
                    {
                        ev.ToFloats(writer);
                    }
                }
            });
            worker.Start();
        }


        public void CaptureCsv(string filename)
        {
            CaptureCsv(new BinaryWriter(new FileStream(filename, FileMode.Create)));
        }

        /// <summary>
        /// Sends all events to a socket at the specified port on the local machine
        /// </summary>
        public void StreamUdp(int port)
        {
            var sendSocket = new Socket((SocketType)AddressFamily.InterNetwork, ProtocolType.Udp);
            var ep = new IPEndPoint(IPAddress.Loopback, port);

            var events = MakeEventQueue();

            var worker = new Thread(() =>
            {
                float[] floats = new float[6];
                byte[] buffer = new byte[sizeof(float) * 6];
                while (true)
                {
                    if (events.TryDequeue(out var ev))
                    {
                        ev.ToFloats(floats);
                        Buffer.BlockCopy(floats, 0, buffer, 0, sizeof(float) * 6);
                        sendSocket.SendTo(buffer, ep);
                    }
                }
            });
            worker.Start();
        }

        static void Main(string[] args)
        {
            Console.WriteLine("Tracker Manager");

            Stopwatch.Start();

            var program = new Program();

            program.Start();

            program.CaptureCsv(@"D:\UCL\TrackerFusion\ImuQuickCapture.bin");
            //program.CaptureCsv(@"C:\Users\Sebastian\Dropbox (Personal)\UCL\Tracker Fusion\Captures\" + "Capture" + ".bin");
            //program.CaptureOpticalCsv(System.Console.Out);
            //program.StreamUdp(SendPort);

            while (true)
            {
                if(Program.Messages.TryDequeue(out var s))
                {
                    Console.WriteLine(s);
                }
            }

            program.Stop();
        }


        // All devices share a common time reference through this stopwatch
        public static Stopwatch Stopwatch = new Stopwatch();
        public static double Time => (double)Program.Stopwatch.ElapsedTicks / (double)Stopwatch.Frequency;
    }

    public interface IEvent
    {
        void ToFloats(BinaryWriter writer);
        void ToFloats(float[] floats);
    }

}