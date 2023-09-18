using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using PhaseSpace.OWL;

namespace TrackerManager
{
    internal class OpticalManager
    {
        private List<OpticalTracker> opticalDevices = new List<OpticalTracker>();
        private Context context;
        private Thread eventThread;

        public delegate void OnOpticalEvent(OpticalEvent ev);
        public event OnOpticalEvent OnEvent;

        public enum Type : int
        {
            Optical = 3,
        }

        public struct OpticalEvent : IEvent
        {
            public int markerId;
            public double time;
            public Vector3 position;

            public override string ToString()
            {
                return $"{time}, {markerId}, {position.X}, {position.Y}, {position.Z}";
            }

            public void ToFloats(BinaryWriter writer)
            {
                writer.Write((float)Type.Optical);
                writer.Write((float)markerId);
                writer.Write((float)time);
                writer.Write(position.X);
                writer.Write(position.Y);
                writer.Write(position.Z);
            }
        }

        public OpticalManager(string hostname)
        {
            context = new Context();
            int result = context.open(hostname);
            if (result <= 0)
            {
                throw new Exception("Cannot connect to " + hostname + " " + result);
            }

            result = context.initialize("streaming=1");
            if (result <= 0)
            {
                throw new Exception("Cannot start streaming from " + hostname + " " + result);
            }

            eventThread = new Thread(EventWorker);
            eventThread.Start();
        }

        public void Dispose()
        {
            context.close();
            eventThread.Join();
        }

        private void EventWorker()
        {
            Program.Log("Connected PhaseSpace");

            while(context.isOpen() && context.property<int>("initialized") > 0)
            {
                var ev = context.nextEvent(1000);
                if (ev != null)
                {
                    if(OnEvent != null)
                    {
                        switch (ev.type_id)
                        {
                            case PhaseSpace.OWL.Type.EVENT:
                                {
                                    foreach (var subevent in ev.subevents)
                                    {
                                        switch (subevent.type_id)
                                        {
                                            case PhaseSpace.OWL.Type.MARKER:
                                                {
                                                    var markers = subevent.data as PhaseSpace.OWL.Marker[];
                                                    foreach (var marker in markers)
                                                    {
                                                        if(marker.cond > 0)
                                                        {
                                                            OnEvent.Invoke(new OpticalEvent()
                                                            {
                                                                position = new Vector3(
                                                                    marker.x,
                                                                    marker.y,
                                                                    marker.z
                                                                    ),
                                                                markerId = (int)marker.id,
                                                                time = Program.Time
                                                            });
                                                        }
                                                    }
                                                }
                                                break;
                                        }
                                    }
                                };
                                break;
                        }
                    }
                }
            }
        }
    }

    public class OpticalTracker
    {
    
    }

}
