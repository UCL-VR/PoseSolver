using System.Collections.Concurrent;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;

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

        private OpticalManager opticalManager; //NB marker 1 is the one without an IMU
        private InertialManager inertialManager;

        public void Start()
        {
            inertialManager = new InertialManager(InertialPorts);            
            //opticalManager = new OpticalManager(phaseSpaceAddress);
        }

        public void Stop()
        {
            inertialManager.Dispose();
            opticalManager.Dispose();
        }

        public static void Log(string m)
        {
            Messages.Enqueue(m);
        }

        public static ConcurrentQueue<string> Messages = new ConcurrentQueue<string>();

        public ConcurrentQueue<IEvent> CreateEventQueue()
        {
            // This is the shared event queue that demultiplexes all event streams

            ConcurrentQueue<IEvent> events = new ConcurrentQueue<IEvent>();

            // This snippet sets up the inertial manager to write to the stream.

            foreach (var item in inertialManager.Devices)
            {
                item.OnEvent += (ev) =>
                {
                    events.Enqueue(ev);
                };
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
        public void CaptureStream(BinaryWriter writer)
        {
            var events = CreateEventQueue();

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

        public void StreamFloats(int port)
        {
            var events = CreateEventQueue();
            var socket = new UdpClient();
            socket.Connect(new IPEndPoint(IPAddress.Loopback, port));
            var floats = new float[6];
            var bytes = new byte[6 * 4];

            var worker = new Thread(() =>
            {
                while(true)
                {
                    if (events.TryDequeue(out var ev))
                    {
                        ev.ToFloats(floats);
                        Buffer.BlockCopy(floats, 0, bytes, 0, bytes.Length);
                        socket.Send(bytes, bytes.Length);
                    }
                }
            });
            worker.Start();
        }

        public void CaptureFile(string filename)
        {
            CaptureStream(new BinaryWriter(new FileStream(filename, FileMode.Create)));
        }

        static void Main(string[] args)
        {
            Console.WriteLine("Tracker Manager");

            Stopwatch.Start();

            var program = new Program();

            program.Start();

            //program.CaptureInertialCsv(System.Console.Out);
            //program.CaptureFile(@"D:\UCL\TrackerFusion\Matlab\Captures\" + "IMU_Static" + ".bin");
            //program.CaptureOpticalCsv(System.Console.Out);
            program.StreamFloats(24693);

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
        void ToFloats(float[] buffer);
    }

}