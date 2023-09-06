/***
Copyright (c) PhaseSpace, Inc 2017

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL PHASESPACE, INC
BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


libowl2 API C# Implementation
PhaseSpace, Inc. 2017

To run in Ubuntu:
    sudo apt-get install mono-complete
    mcs -debug owl.cs
    mono --debug owl.exe

Changelog
=========
* 5.2.386 (2017-09-05)
  * fixed an issue where changes to initial session properties could become stale.
* 5.2.379 (2017-08-17)
  * added Context.findTrackerInfo()
  * added Context.findDeviceInfo()
  * added Context.deviceOptions()
* 5.2.364 (2017-08-02)
  * bumped version to API version 5.2
  * packing behaviour updated to 5.2.
  * properties received during open() are now preserved between sessions after done().
  * requesting a nonexistent property via Context.property will now return null or default(T)
  * refactored the way persistent properties are saved between inits.
  * Changed OWL.Plane.distance to OWL.Plane.offset
  * moved try_parse to OWL.utils class.
  * Split utils.try_parse into utils.parse_number
  * Changed DeviceInfo id to an int
  * Fixed typo in key compare for profiles.json.
  * removed some extraneous code
  * slight refactoring.


***/

using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Linq;

//
namespace PhaseSpace.OWL
{
    //
    using stringmap = Dictionary<string, string>;

    /// <summary>
    /// TODO
    /// </summary>
    public class Version
    {
        static readonly public string OWL_PROTOCOL_VERSION = "2";
        static readonly public string OWL_VERSION = "5.2.401";
    }

    //
    public class OWLError : System.Exception
    {
        public OWLError(string msg) : base(msg) { }
    }

    //
    public enum Type
    {
        INVALID = 0,
        BYTE,
        STRING = BYTE,
        INT,
        FLOAT,
        ERROR = 0x7F,
        EVENT = 0x80,
        FRAME = EVENT,
        CAMERA,
        PEAK,
        PLANE,
        MARKER,
        RIGID,
        INPUT,
        MARKERINFO,
        TRACKERINFO,
        FILTERINFO,
        DEVICEINFO,
        PACKINFO
    }

    //
    public class Camera
    {
        public uint id;
        public uint flags;
        public float[] pose = new float[7];
        public float cond;

        //
        public override string ToString()
        {
            return string.Format("Camera(id={0}, flags={1}, pose={{{2}, {3}, {4}, {5}, {6}, {7}, {8}}}, cond={9})", id, flags, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], cond);
        }
    }

    //
    public class Peak
    {
        public uint id;
        public uint flags;
        public long time;
        public uint camera;
        public uint detector;
        public uint width;
        public float pos;
        public float amp;

        //
        public override string ToString()
        {
            return string.Format("Peak(id={0}, flags={1}, time={2}, camera={3}, detector={4}, width={5}, pos={6}, amp={7})", id, flags, time, camera, detector, width, pos, amp);
        }
    }

    //
    public class Plane
    {
        public uint id;
        public uint flags;
        public long time;
        public ushort camera;
        public ushort detector;
        public float[] plane = new float[4];

        public float offset;

        //
        public override string ToString()
        {
            return string.Format("Plane(id={0}, flags={1], time={2}, camera={3}, detector={4}, plane={{5}, {6}, {7}, {8}}, offset={9})", id, flags, time, camera, detector, plane[0], plane[1], plane[2], plane[3], offset);
        }
    };

    //
    public class Marker
    {
        public uint id;
        public uint flags;
        public long time;
        public float x, y, z;
        public float cond;

        //
        public override string ToString()
        {
            return string.Format("Marker(id={0}, flags={1}, time={2}, x={3}, y={4}, z={5}, cond={6})", id, flags, time, x, y, z, cond);
        }
    }

    //
    public class Rigid
    {
        public uint id;
        public uint flags;
        public long time;
        public float[] pose = new float[7];
        public float cond;

        //
        public override string ToString()
        {
            return string.Format("Rigid(id={0}, flags={1}, time={2}, pose={{{3}, {4}, {5}, {6}, {7}, {8}, {9}}}, cond={10})", id, flags, time, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], cond);
        }
    }

    //
    public class Input
    {
        public ulong hw_id = 0;
        public ulong flags = 0;
        public long time = 0;
        public byte[] data = null;

        //
        public override string ToString()
        {
            int n = 8;
            if (n > data.Length) n = data.Length;
            string _data = "";
            for (int i = 0; i < n; i++)
                _data += string.Format("0x{0:X02}, ", data[i]);
            return string.Format("Input(hw_id={0}, flags={1}, time={2}, data=\"{3}... ({4} bytes)\")", hw_id, flags, time, _data, data.Length);
        }
    }

    //
    public class MarkerInfo
    {
        public uint id;
        public uint tracker_id;
        public string name;
        public string options;

        //
        public MarkerInfo(uint id = 0xffffffff, uint tracker_id = 0xffffffff, string name = "", string options = "")
        {
            this.id = id;
            this.tracker_id = tracker_id;
            this.name = name;
            this.options = options;
        }

        //
        public override string ToString()
        {
            return string.Format("MarkerInfo(id={0} tracker={1} name=\"{2}\" options=\"{3}\")", id, tracker_id, name, options);
        }

    }

    //
    public class TrackerInfo
    {
        public uint id;
        public string type;
        public string name;
        public string options;
        public uint[] marker_ids;

        //
        public TrackerInfo(uint id = 0xffffffff, string type = "", string name = "", string options = "", uint[] marker_ids = null)
        {
            this.id = id;
            this.type = type;
            this.name = name;
            this.options = options;
            if (marker_ids != null)
                this.marker_ids = (uint[])marker_ids.Clone();
            else if (marker_ids == null)
                this.marker_ids = new uint[0];
        }

        //
        public TrackerInfo(uint id, string type, string name, string options, string marker_ids)
        {
            this.id = id;
            this.type = type;
            this.name = name;
            this.options = options;
            utils.get<uint>(marker_ids, out this.marker_ids);
        }

        //
        public override string ToString()
        {
            return string.Format("TrackerInfo(id={0} type={1} name=\"{2}\" options=\"{3}\" marker_ids={4})", id, type, name, options, string.Join(",", marker_ids.Select(m => m.ToString()).ToArray()));
        }
    }

    //
    public class FilterInfo
    {
        public uint period;
        public string name = "";
        public string options = "";

        //
        public override string ToString()
        {
            return string.Format("FilterInfo(period={0}, name=\"{1}\", options=\"{2}\")", period, name, options);
        }
    }

    //
    public class DeviceInfo
    {
        public ulong hw_id;
        public long time;
        public string type;
        public string name;
        public string options;
        public string status;

        //
        public DeviceInfo(ulong hw_id = 0, long time = -1, string type = "", string name = "")
        {
            this.hw_id = hw_id;
            this.time = -1;
            this.type = type;
            this.name = name;
            this.options = "";
            this.status = "";
        }

        //
        public override string ToString()
        {
            return string.Format("DeviceInfo(hw_id=0x{0:X16}, time={1}, type={2}, name=\"{3}\", options=\"{4}\", status=\"{5}\")", hw_id, time, type, name, options, status);
        }
    }

    //
    public class PackInfo
    {
        public Type type_id;
        public ushort id;
        public string name;
        public string options;
        public ulong [] ids;

        //
        public PackInfo(Type type_id = 0, string name = "", string options = "", ulong [] ids = null)
        {
            this.type_id = type_id;
            this.id = ushort.MaxValue;
            this.name = name.Trim();
            this.options = options.Trim();
            if(ids == null) ids = new ulong[0];
            this.ids = ids;
        }

        //
        public PackInfo(Type type_id, string name, string options, string ids)
        {
            this.type_id = type_id;
            this.id = ushort.MaxValue;
            this.name = name.Trim();
            this.options = options.Trim();
            utils.get<ulong>(ids, out this.ids);
        }

        //
        public override bool Equals(System.Object o)
        {
            if(o == null || !(o is PackInfo)) return false;
            PackInfo p = o as PackInfo;
            return this.type_id == p.type_id && this.id == p.id && this.name == p.name && this.options == p.options && this.ids.SequenceEqual(p.ids);
        }

        //
        public override int GetHashCode()
        {
            return this.id;
        }

        //
        public override string ToString()
        {
            return string.Format("PackInfo(type_id={0}, id={1}, name={2}, {3}, ids={4})",
                                 type_id, id, name, options,
                                 string.Join(",", ids.Select(x => x.ToString()).ToArray()));
        }
    }

    //
    public class Event
    {
        public Type type_id;
        public ushort id;
        public uint flags;
        public long time;
        public string type_name;
        public string name;
        public object data;

        public List<Event> subevents;

        //
        public object this[string name]
        {
            get
            {
                if(this.name == name) return data;
                if(subevents != null)
                {
                    for(int i = 0; i < subevents.Count; i++)
                    {
                        if(subevents[i].name == name)
                            return subevents[i].data;
                    }
                }
                return null;
            }
        }

        //
        public bool find<T>(string name, out T v)
            where T : class
                      {
                          System.Type t = typeof(T);
                          if(this.name.Length == 0 || this.name == name)
                          {
                              if(data != null && data.GetType() == t)
                              {
                                  v = (T)System.Convert.ChangeType(data, t);
                                  return true;
                              }
                          }
                          if(subevents != null)
                          {
                              for(int i = 0; i < subevents.Count; i++)
                              {
                                  if(subevents[i].find<T>(name, out v))
                                      return true;
                              }
                          }
                          v = null;
                          return false;
            }

        //
        public void add(Event e)
        {
            if(subevents == null) subevents = new List<Event>();
            subevents.Add(e);
        }

        //
        public Event(Type type_id, ushort id, uint flags, long time, string type_name, string name, object data = null)
        {
            this.type_id = type_id;
            this.id = id;
            this.flags = flags;
            this.time = time;
            this.type_name = type_name;
            this.name = name;
            this.data = data;
        }

        //
        public override string ToString()
        {
            string dtype = (data == null) ? "(null)" : data.GetType().ToString();
            string names = "(null)";
            if(subevents != null) names = string.Join(",", subevents.Select(x => x.name).ToArray());
            return string.Format("Event(type_id={0} id={1} flags={2} time={3} type_name={4} name={5} data={6} subevents={7})", type_id, id, flags, time, type_name, name, dtype, names);
        }
    }

    //
    public class Context
    {
        // print debugging output
        public bool debug = false;

        // for internal testing
        public int flag = 0;
        public int ReceiveBufferSize = 16*1024*1024;
        public Stats stats;

        //
        Sockets sockets = new Sockets();

        // open state
        protected int port_offset = 0;
        protected int connect_state = 0;

        // receive buffers
        Protocol protocol;
        ProtocolUdp protocol_udp;

        // session state
        protected Dictionary<string, object> properties = new Dictionary<string, object>();
        protected Dictionary<string, object> initial = new Dictionary<string, object>();
        protected Dictionary<uint, TypeInfo> types = new Dictionary<uint, TypeInfo>();
        protected Dictionary<uint, TypeInfo> names = new Dictionary<uint, TypeInfo>();
        protected Dictionary<uint, TrackerInfo> trackers = new Dictionary<uint, TrackerInfo>();
        protected Dictionary<uint, MarkerInfo> markers = new Dictionary<uint, MarkerInfo>();
        protected Dictionary<ulong, DeviceInfo> devices = new Dictionary<ulong, DeviceInfo>();
        protected Dictionary<string, FilterInfo> _filters = new Dictionary<string, FilterInfo>();
        protected PackInfo [] packinfo = new PackInfo[0];
        protected stringmap _options = new stringmap();

        protected Queue<Event> events = new Queue<Event>();
        protected Event newFrame = null;

        public System.Exception lastException;

        Timer dbgtimer = new Timer();

        //
        protected class TypeInfo
        {
            public uint flags;
            public int mode;
            public string name;

            public override string ToString()
            {
                return string.Format("TypeInfo(flags={0} mode={1} name={2})", flags, mode, name);
            }
        }

        // ctor
        public Context()
        {
            protocol = new Protocol(ReceiveBufferSize);
            protocol_udp = new ProtocolUdp(ReceiveBufferSize);
            dbgtimer.Start();

            clear();
        }

        // get most recent error message
        public string lastError()
        {
            if(lastException == null) return null;
            string msg =  lastException.Message;
            if(lastException.StackTrace != null) msg = string.Format("{0}\n{1}", msg, lastException.StackTrace);
            return msg;
        }

       // print debugging output
        protected void log(object msg)
        {
            if (!debug) return;
            if (msg == null) msg = "(null)";
            else msg = string.Format("[{0,12}] {1}", dbgtimer.ElapsedNanoseconds, msg);
            utils.log(msg);
        }

        //
        protected void clear(bool flag = false)
        {
            log("clear()");

            if(!flag) prop_merge(initial, properties, false);

            properties.Clear();

            properties["apiversion"] = "";
            properties["opened"] = 0;
            properties["initialized"] = 0;
            properties["streaming"] = 0;
            properties["name"] = "";
            properties["profile"] = "";
            properties["local"] = 0;
            properties["systemtimebase"] = new int[2] { 0, 0 };
            properties["timebase"] = new int[2] { 0, 0 };
            properties["maxfrequency"] = 960.0f;
            properties["systemfrequency"] = 0.0f;
            properties["frequency"] = 0.0f;
            properties["scale"] = 1.0f;
            properties["slave"] = 0;
            properties["systempose"] = new float[7] { 0, 0, 0, 1, 0, 0, 0 };
            properties["pose"] = new float[7] { 0, 0, 0, 1, 0, 0, 0 };
            properties["options"] = ""; // memoized status string of _options
            properties["systemcameras"] = new Camera[0];
            properties["missingsystemcameras"] = new Camera[0];
            properties["cameras"] = new Camera[0];
            properties["missingcameras"] = new Camera[0];
            properties["markers"] = 0;
            properties["markerinfo"] = new MarkerInfo[0];
            properties["trackers"] = new uint[0];
            properties["trackerinfo"] = new TrackerInfo[0];
            properties["filters"] = new string[0];
            properties["filterinfo"] = new FilterInfo[0];
            properties["deviceinfo"] = new DeviceInfo[0];
            properties["packinfo"] = new PackInfo[0];
            properties["profiles"] = new string[0];
            properties["defaultprofile"] = "";
            properties["profiles.json"] = "";

            newFrame = null;

            trackers = new Dictionary<uint, TrackerInfo>();
            markers = new Dictionary<uint, MarkerInfo>();
            devices = new Dictionary<ulong, DeviceInfo>();
            _filters = new Dictionary<string, FilterInfo>();
            _options = new stringmap();
            packinfo = new PackInfo[0];

            connect_state = 0;

            events.Clear();

            if(flag)
            {
                initial = properties.ToDictionary(entry => entry.Key, entry => entry.Value);
            }
            else
            {
                prop_merge(properties, initial, true);
            }
        }

        //
        public int open(string name, string open_options = "")
        {
            try
            {
                log(string.Format("open({0}, {1})", name, open_options));
                if (property<int>("opened") == 1)
                {
                    log(string.Format("    already open."));
                    return 1;
                }

                // parse options
                stringmap opts = utils.tomap(open_options);
                int timeout_usec = 10000000;
                if (opts.ContainsKey("timeout"))
                    utils.try_parse(opts["timeout"], out timeout_usec);

                // connect to server
                if (connect_state == 0)
                {
                    clear(true);
                    properties["opening"] = 1;

                    // parse host and port offset
                    const int DEFAULT_PORT = 8000;
                    port_offset = 0;
                    string[] hp = name.Split(':');
                    if (hp.Length > 1) utils.try_parse(hp[1], out port_offset);
                    name = hp[0];
                    int port = DEFAULT_PORT + port_offset;
                    properties["name"] = name;
                    sockets.tcp = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                    // configure socket
                    sockets.tcp.NoDelay = true;
                    sockets.tcp.ReceiveBufferSize = ReceiveBufferSize;

                    // lookup remote host
                    IPAddress[] ips = Dns.GetHostAddresses(name);
                    if (ips.Length <= 0) throw new OWLError("unable to resolve hostname");
                    IPEndPoint ep = new IPEndPoint(ips[0], port);

                    // start asynchronous connect
                    log("    connecting to " + name + " (" + ips[0].ToString() + ":" + port + ")");
                    connect_state++;
                    try
                    {
                        sockets.tcp.BeginConnect(ep,
                            delegate(System.IAsyncResult ar)
                            {
                                Context c = (Context)ar.AsyncState;
                                try
                                {
                                    c.sockets.tcp.EndConnect(ar);
                                }
                                catch (System.ObjectDisposedException)
                                {
                                    log("socket connect interrupted");
                                }
                                catch (SocketException e)
                                {
                                    log("socket exception: " + e.Message);
                                }
                            }
                            , this);
                    }
                    catch (SocketException e)
                    {
                        // connect needs more time
                        if (e.SocketErrorCode != SocketError.WouldBlock)
                            throw e;
                    }
                }

                // wait for socket to be ready
                if (connect_state == 1)
                {
                    wait_delegate d = delegate () {
                        return sockets.tcp.Connected;
                    };

                    if (wait(timeout_usec, d))
                    {
                        int err = utils.GetSocketError(sockets.tcp);
                        if (err != (int)SocketError.Success)
                            throw new SocketException(err);
                        connect_state++;
                    }
                }

                if (connect_state == 2)
                {
                    sockets.tcp.Blocking = false;
                    log("    waiting for server state");
                    connect_state++;
                }

                // receive state from server
                if (connect_state == 3)
                {
                    wait_delegate d = delegate () {
                        if(recv(0) < 0) throw lastException;
                        return property<int>("opened") == 1;
                    };
                    if (wait(timeout_usec, d))
                    {
                        int err = utils.GetSocketError(sockets.tcp);
                        if (err != (int)SocketError.Success)
                            throw new SocketException(err);
                        connect_state++;
                    }
                }

                if (property<int>("opened") == 1)
                {
                    log("    success");
                    properties.Remove("opening");
                    lastException = null;

                    if(connect_state == 4)
                    {
                        // upload version info
                        if (!send(Type.BYTE, "internal", string.Format("protocol={0} libowl={1}", Version.OWL_PROTOCOL_VERSION, Version.OWL_VERSION)))
                            throw new OWLError("version upload fail");
                        connect_state++;

                        // tcp connection established, open udp receive socket
                        try
                        {
                            sockets.udp = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                            sockets.udp.Blocking = false;
                            sockets.udp.ReceiveBufferSize = ReceiveBufferSize;
                            sockets.udp.ExclusiveAddressUse = false;

                            IPEndPoint lep = sockets.tcp.LocalEndPoint as IPEndPoint;
                            IPEndPoint ep = new IPEndPoint(IPAddress.Any, lep.Port);
                            log(string.Format("binding to udp port {0}", lep.Port));
                            sockets.udp.Bind(ep);
                        }
                        catch(System.Exception e)
                        {
                            lastException = e;
                            log(e);
                            sockets.udp = null;
                        }
                    }

                    // save initial properties
                    initial = prop_diff(initial);

                    return 1;
                }

                if (timeout_usec == 0) return 0;
                throw new OWLError("connection timed out (state: " + connect_state + ")");

            }
            catch (System.Exception e)
            {
                if(properties.ContainsKey("opening")) properties.Remove("opening");
                lastException = e;
                log(e);
                close();
                return -1;
            }
        }

        //
        public bool close()
        {
            log("close()");
            try
            {
                // shutdown tcp socket
                if (sockets.tcp.Connected)
                {
                    sockets.tcp.Shutdown(SocketShutdown.Both);
                }
                sockets.tcp.Close();

                // shutdown udp sockets
                if (sockets.bcast != null)
                {
                    sockets.bcast.Close();
                    sockets.bcast = null;
                }
                if (sockets.udp != null)
                {
                    sockets.udp.Close();
                    sockets.udp = null;
                }
            }
            catch (System.Exception e)
            {
                lastException = e;
                log(e);
                return false;
            }
            finally
            {
                initial.Clear();
                properties.Clear();
                clear();
            }
            return true;
        }

        //
        public bool isOpen()
        {
            return sockets.tcp.Connected && property<int>("opened") == 1;
        }

        //
        public int initialize(string init_options = "")
        {
            try
            {
                log(string.Format("initialize({0})", init_options));
                if (!isOpen()) throw new OWLError("no connection");
                if (property<int>("initialized") == 1)
                {
                    log(string.Format("    already initialized"));
                    return 1;
                }

                // parse options
                stringmap opts = utils.tomap(init_options);
                int timeout_usec = 10000000;
                if (opts.ContainsKey("timeout"))
                {
                    utils.try_parse(opts["timeout"], out timeout_usec);
                }

                if (!properties.ContainsKey("initializing"))
                {
                    clear();
                    properties["initializing"] = 1;

                    log(string.Format("    uploading options"));

                    // tell server to initialize
                    if (!send(Type.BYTE, "initialize", "event.raw=1 event.markers=1 event.rigids=1 event.info=1" + (init_options.Length > 0 ? " " : "") + init_options))
                        return -1;

                    if (timeout_usec == 0) return 0;
                    log(string.Format("    waiting for response"));
                }

                wait_delegate d = delegate () {
                    if(recv(0) < 0) throw lastException;
                    return property<int>("initialized") == 1;
                };

                // wait for server state
                if (!wait(timeout_usec, d) && timeout_usec == 0)
                    return 0;

                if (property<int>("initialized") == 0)
                {
                    log("    failed");
                    if (timeout_usec > 0)
                        throw new OWLError("timed out");
                    throw new OWLError("init failed");
                }

                if(flag == 1) throw new OWLError("test");
            }
            catch (System.Exception e)
            {
                properties.Remove("initializing");
                properties["initialized"] = 0;
                log(e);
                lastException = e;
                return -1;
            }
            log(string.Format("    success"));
            lastException = null;
            return 1;
        }

        //
        public int done(string done_options = "")
        {
            log(string.Format("done({0})", done_options));
            if (!isOpen())
            {
                lastException = new OWLError("error: Closed context");
                return -1;
            }

            stringmap opts = utils.tomap(done_options);
            long timeout_usec = 10000000;
            if(opts.ContainsKey("timeout"))
                if(!utils.try_parse(opts["timeout"], out timeout_usec))
                    throw new OWLError("option parse error");
            if(property<int>("initialized") == 0)
            {
                if(properties.ContainsKey("flushing"))
                    goto finish;
                return 1;
            }
            if(!properties.ContainsKey("flushing"))
            {
                properties["flushing"] = 1;
                // inform server that this session is done
                if(!send(Type.BYTE, "done", done_options))
                    return -1;
            }

            try
            {
                // wait for "initialized" == 0
                wait(timeout_usec, delegate() {
                        if(flag == 1) return false;
                        if(recv(0) < 0) throw lastException;
                        return property<int>("initialized") == 0;
                    });
            }
            catch(System.Exception e)
            {
                lastException = e;
                log(e);
                return -1;
            }

            if(property<int>("initialized") == 1)
            {
                if(timeout_usec > 0)
                {
                    lastException = new OWLError("error: timed out");
                    log(lastException);
                    return -1;
                }
                return 0;
            }
        finish:
            if(properties.ContainsKey("flushing"))
                properties.Remove("flushing");
            return 1;
        }

        //
        public int streaming()
        {
            return property<int>("streaming");
        }

        //
        public bool streaming(int enable)
        {
            return send(Type.INT, "streaming", new int[1]{enable});
        }

        //
        public float frequency()
        {
            return property<float>("frequency");
        }

        //
        public bool frequency(float freq)
        {
            return send(Type.FLOAT, "frequency", new float[1]{freq});
        }

        //
        public int[] timeBase()
        {
            return property<int[]>("timebase");
        }

        //
        public bool timeBase(int numerator, int denominator)
        {
            return send(Type.INT, "timebase", new int[2]{numerator, denominator});
        }

        //
        public float scale()
        {
            return property<float>("scale");
        }

        //
        public bool scale(float s)
        {
            return send(Type.FLOAT, "scale", new float[1] { s });
        }

        //
        public float[] pose()
        {
            return property<float[]>("pose");
        }

        //
        public bool pose(float[] pose)
        {
            return send(Type.FLOAT, "pose", pose);
        }

        //
        public string option(string optname)
        {
            return _options[optname];
        }

        //
        public bool option(string optname, string value)
        {
            return send(Type.BYTE, "options", string.Format("{0}={1}", optname, value));
        }

        //
        public string options()
        {
            return property<string>("options");
        }

        //
        public bool options(string options)
        {
            return send(Type.BYTE, "options", options);
        }

        //
        public bool createTracker(uint tracker_id, string tracker_type, string tracker_name = "", string tracker_options = "")
        {
            return createTrackers(new TrackerInfo[1] { new TrackerInfo(tracker_id, tracker_type, tracker_name, tracker_options) });
        }

        //
        public bool createTrackers(TrackerInfo[] trackers)
        {
            System.IO.StringWriter tw = new System.IO.StringWriter();
            string pad = "";
            foreach (TrackerInfo ti in trackers)
            {
                tw.Write(string.Format("{0}id={1} type={2} name={3}", pad, ti.id, ti.type, ti.name));
                if (ti.marker_ids.Length > 0)
                {
                    tw.Write(" mid=");
                    tw.Write(string.Join(",", ti.marker_ids.Select(x => string.Format("{0}", x)).ToArray()));
                }
                if (ti.options.Length > 0)
                {
                    tw.Write(' ');
                    tw.Write(ti.options);
                }
                pad = " ";
            }
            return send(Type.BYTE, "createtracker", tw.ToString());
        }

        //
        public bool trackerName(uint tracker_id, string tracker_name)
        {
            return send(Type.BYTE, "trackername", string.Format("id={0} name={1}", tracker_id, tracker_name));
        }

        //
        public bool trackerOptions(uint tracker_id, string tracker_options)
        {
            return send(Type.BYTE, "trackeroptions", string.Format("id={0} {1}", tracker_id, tracker_options));
        }

        // TODO test
        public TrackerInfo [] findTrackerInfo(string type, string name = "")
        {
            List<TrackerInfo> tr = new List<TrackerInfo>();
            foreach(TrackerInfo t in trackers.Values)
            {
                if((type == "" || type == t.type) && (name == "" || name == t.name))
                {
                    tr.Add(t);
                }
            }
            return tr.ToArray();
        }

        //
        public TrackerInfo trackerInfo(uint tracker_id)
        {
            if (trackers.ContainsKey(tracker_id))
                return trackers[tracker_id];
            return null;
        }

        //
        public bool destroyTracker(uint tracker_id)
        {
            return send(Type.BYTE, "destroytracker", string.Format("id={0}", tracker_id));
        }

        //
        public bool destroyTrackers(uint[] tracker_ids)
        {
            return send(Type.BYTE, "destroytracker", string.Join(" ", tracker_ids.Select(x => string.Format("id={0}", x)).ToArray()));
        }

        //
        public bool assignMarker(uint tracker_id, uint marker_id, string marker_name = "", string marker_options = "")
        {
            return assignMarkers(new MarkerInfo[1] { new MarkerInfo(marker_id, tracker_id, marker_name, marker_options) });
        }

        //
        public bool assignMarkers(MarkerInfo[] markers)
        {
            System.IO.StringWriter tw = new System.IO.StringWriter();
            string pad = "";
            foreach (MarkerInfo mi in markers)
            {
                tw.Write("{0}tid={1} mid={2} name={3}{4}{5}", pad, mi.tracker_id, mi.id, mi.name, mi.options.Length > 0 ? " " : "", mi.options);
                pad = " ";
            }
            return send(Type.BYTE, "assignmarker", tw.ToString());
        }

        //
        public bool markerName(uint marker_id, string marker_name)
        {
            return send(Type.BYTE, "markername", string.Format("mid={0} name={1}", marker_id, marker_name));
        }

        //
        public bool markerOptions(uint marker_id, string marker_options)
        {
            return send(Type.BYTE, "markeroptions", string.Format("mid={0} {1}", marker_id, marker_options));
        }

        //
        public MarkerInfo markerInfo(uint marker_id)
        {
            if (markers.ContainsKey(marker_id))
                return markers[marker_id];
            return null;
        }

        //
        public bool filter(uint period, string name, string filter_options)
        {
            return send(Type.BYTE, "filter", string.Format("filter={0} period={1}{2}{3}", name, period, filter_options.Length > 0 ? " " : "", filter_options));
        }

        //
        public bool filters(FilterInfo[] filters)
        {
            System.IO.StringWriter tw = new System.IO.StringWriter();
            string pad = "";
            foreach (FilterInfo fi in filters)
            {
                tw.Write(string.Format("{0}filter={1} period={2}{3}{4}", pad, fi.name, fi.period, fi.options.Length > 0 ? " " : "", fi.options));
                pad = " ";
            }
            return send(Type.BYTE, "filter", tw.ToString());
        }

        //
        public FilterInfo filterInfo(string name)
        {
            if (_filters.ContainsKey(name))
                return _filters[name];
            return null;
        }

        //
        public bool deviceOptions(ulong hw_id, string device_options)
        {
            string o = string.Format("hwid=0x{0:x} {1}", hw_id, device_options);
            log(string.Format("send deviceoptions: {0}", o));
            return send(Type.BYTE, "deviceoptions", o);
        }

        //  TODO test
        public DeviceInfo [] findDeviceInfo(string type, string name = "")
        {
            List<DeviceInfo> dr = new List<DeviceInfo>();
            foreach(DeviceInfo d in devices.Values)
            {
                if((type == "" || type == d.type) && (name == "" || name == d.name))
                {
                    dr.Add(d);
                }
            }
            return dr.ToArray();
        }

        //
        public DeviceInfo deviceInfo(ulong hw_id)
        {
            if (devices.ContainsKey(hw_id))
                return devices[hw_id];
            return null;
        }

        //
        public PackInfo [] packInfo()
        {
            return packinfo;
        }

        //
        public Event peekEvent(int timeout_usec = 0)
        {
            recv(events.Count > 0 ? 0 : timeout_usec);
            if (events.Count > 0)
                return events.Peek();
            return null;
        }

        //
        public Event nextEvent(int timeout_usec = 0)
        {
            recv(events.Count > 0 ? 0 : timeout_usec);
            if (events.Count > 0)
                return events.Dequeue();
            return null;
        }

        //
        public object property(string name)
        {
            try
            {
                return properties[name];
            }
            catch(System.Collections.Generic.KeyNotFoundException)
            {
                return null;
            }
        }

        //
        public T property<T>(string name)
        {
            try
            {
                return (T)System.Convert.ChangeType(properties[name], typeof(T));
            }
            catch(System.Collections.Generic.KeyNotFoundException)
            {
                return default(T);
            }
        }

        //
        public bool pack(PackInfo [] packinfo)
        {
            if(property<string>("apiversion") == "")
                throw new OWLError("packing not supported");
            if(packinfo.Length <= 0) return true;
            System.IO.StringWriter w = new System.IO.StringWriter();
            string pad = "";
            for(int i = 0; i < packinfo.Length; i++)
            {
                PackInfo pi = packinfo[i];
                w.Write(string.Format("{0}type={1} name={2}", pad, (ushort)pi.type_id, pi.name));
                if(pi.ids.Length > 0)
                {
                    w.Write(" ids=");
                    w.Write(string.Join(",", pi.ids.Select(x => string.Format("{0}", x)).ToArray()));
                }
                if(pi.options.Length > 0)
                {
                    w.Write(" ");
                    w.Write(pi.options);
                }
                pad = " ";
            }
            utils.log(string.Format("pack({0})", w.ToString()));
            return send(Type.BYTE, "pack", w.ToString());
        }


        //internal functions
        protected Dictionary<string, object> prop_diff(Dictionary<string, object> p) // this != p
        {
            Dictionary<string, object> r = new Dictionary<string, object>();
            foreach(KeyValuePair<string, object> i in properties)
            {
                if(!p.ContainsKey(i.Key) || p[i.Key] != i.Value)
                {
                    r[i.Key] = i.Value;
                }
            }
            return r;
        }

        // full ? a += b : b in a = b
        protected void prop_merge(Dictionary<string, object> a,
                                  Dictionary<string, object> b,
                                  bool full = true)
        {
            if(full)
            {
                foreach(KeyValuePair<string, object> i in b)
                {
                    a[i.Key] = i.Value;
                }
            }
            else // b in a = b
            {
                foreach(KeyValuePair<string, object> i in a.ToArray())
                {
                    if(b.ContainsKey(i.Key)) a[i.Key] = b[i.Key];
                }
            }
        }

        //
        int recv_helper(Socket sock, Protocol p)
        {

            //iterate multiple times because some frames are comprised of multiple events
            int maxiter = 6;
            int ret = 0;
            for (int i = 0; i < maxiter; i++)
            {
                Event [] events = p.recv(sock);
                if (events == null) break;
                for(int j = 0; j < events.Length; j++)
                {
                    ret += process_event(events[j]);
                }
            }
            return ret;
        }

        //
        protected int recv(int timeout_usec)
        {
            List<Socket> rsocks = sockets.all;

            try
            {
                Socket.Select(rsocks, null, null, timeout_usec);
                if(rsocks.Count == 0) return 0;

                // tcp
                if(rsocks.Contains(sockets.tcp))
                    stats.tcp_packet_count += recv_helper(sockets.tcp, protocol);
                // udp
                if(rsocks.Contains(sockets.udp))
                    stats.udp_packet_count += recv_helper(sockets.udp, protocol_udp);
                // udp broadcast
                if(rsocks.Contains(sockets.bcast))
                    stats.udp_bcast_packet_count += recv_helper(sockets.bcast, protocol_udp);
            }
            catch (System.Exception e)
            {
                lastException = e;
                log(e);
                close();
                return -1;
            }
            return 1;
        }

        //
        protected void fatal_check(Event evt)
        {
            if(evt.type_id == Type.ERROR && (evt.name == "error" || evt.name == "fatal"))
            {
                if(properties.ContainsKey("initializing") ||
                   properties.ContainsKey("opening"))
                    throw new OWLError(utils.UTF8(evt.data as byte[]));
            }
        }

        //
        protected int process_event(Event evt)
        {
            // handle internal events
            if (evt.id == 0)
            {
                fatal_check(evt);
                if (evt.type_id != Type.BYTE)
                {
                    log(string.Format("warning: unknown event: type={0} id={1} name={2}", evt.type_id, evt.id, evt.name));
                    return 0;
                }
                handle_internal(evt);
                return 0;
            }

            // end-of-frame event
            if (evt.type_id == Type.FRAME && newFrame != null && evt.id == newFrame.id && evt.time == newFrame.time)
            {
                evt = newFrame;
                newFrame = null;
            }

            // accumulate data into frame if part of one, otherwise pass through
            if ((evt.id & 0xff00) != 0)
            {
                ushort frame_id = (ushort)(evt.id >> 8);
                evt.id &= 0xff;
                if (!types.ContainsKey((uint)evt.type_id) || !names.ContainsKey(evt.id))
                {
                    // unknown event, drop it
                    log(string.Format("warning: unknown event: type={0} id={1} name={2}", evt.type_id, evt.id, evt.name));
                }
                else
                {
                    // create a new frame if necessary
                    if (newFrame == null || frame_id != newFrame.id || evt.time != newFrame.time)
                        newFrame = new Event(Type.FRAME, frame_id, 0, evt.time, "", "");
                    // add new event to frame
                    newFrame.add(evt);
                }
                return 0;
            }

            // set type_names and names
            if(!populate_event(evt))
            {
                log("error: name lookup failed");
                return 0;
            }

            // process special cases
            if (evt.type_id == Type.BYTE) handle_byte(evt);
            else if (evt.type_id == Type.FLOAT) handle_float(evt);
            else if (evt.type_id == Type.INT) handle_int(evt);
            else if (evt.type_id == Type.CAMERA) handle_camera(evt);
            else if (evt.type_id == Type.ERROR) fatal_check(evt);

            // send to user
            events.Enqueue(evt);
            return 1;
        }

        //
        protected bool populate_event(Event evt)
        {
            if (!types.ContainsKey((uint)evt.type_id) || !names.ContainsKey(evt.id))
                return false;
            evt.type_name = types[(uint)evt.type_id].name;
            evt.name = names[(uint)evt.id].name;
            if(evt.subevents != null)
            {
                for(int i = 0; i < evt.subevents.Count; i++)
                    if(!populate_event(evt.subevents[i]))
                        return false;
            }
            return true;
        }

        //
        protected static long findID(Dictionary<uint, TypeInfo> table, string name)
        {
            foreach (KeyValuePair<uint, TypeInfo> kv in table)
            {
                if (kv.Value.name == name)
                    return kv.Key;
            }
            return -1;
        }

        //
        protected void update_property(string key, string value)
        {
            log(string.Format("property update: {0}={1}", key, value));
            if(key == "keepalive") return;
            if(properties.ContainsKey(key))
            {
                if(value == "")
                {
                    properties.Remove(key);
                    return;
                }
                System.Type t = properties[key].GetType();
                if (t == typeof(int))
                    properties[key] = System.Int32.Parse(value);
                else if (t == typeof(float))
                    properties[key] = System.Single.Parse(value);
                else if (t == typeof(int[]))
                    properties[key] = value.Split(',').Select(x => System.Int32.Parse(x)).ToArray();
                else if (t == typeof(float[]))
                    properties[key] = value.Split(',').Select(x => System.Int32.Parse(x)).ToArray();
                else properties[key] = value;
            } else {
                properties[key] = value;
            }
        }

        //
        protected bool send<T>(Type type, string name, T[] data) where T:struct
        {
            ushort id = (ushort)findID(names, name);
            if (id < 0) throw new OWLError("invalid name");
            int count = Marshal.SizeOf(typeof(T)) * data.Length;
            byte[] bytes = new byte[count];
            System.Buffer.BlockCopy(data, 0, bytes, 0, count);
            try
            {
                bool ret = protocol.send(sockets.tcp, id, (byte)type, bytes);
                if(!ret) close();
                return ret;
            }
            catch (System.Exception e)
            {
                lastException = e;
                log(e);
                close();
                return false;
            }
        }

        //
        protected bool send(Type type, string name, string data)
        {
            byte[] bytes = System.Text.Encoding.ASCII.GetBytes(data);
            return send(type, name, bytes);
        }

        //
        protected void handle_internal(Event e)
        {
            if (e.data == null)
                throw new OWLError("null internal data");
            string data = utils.UTF8(e.data as byte[]);
            log(data);
            if (data.StartsWith("table=types"))
            {
                parseType(types, data);
            }
            else if (data.StartsWith("table=names"))
            {
                parseType(names, data);
            }
            else if (data.StartsWith("table=trackers"))
            {
                parseTrackerInfo(trackers, data);
                uint[] t = trackers.Values.Select(x => x.id).ToArray();
                TrackerInfo[] ti = trackers.Values.ToArray();
                properties["trackers"] = t;
                properties["trackerinfo"] = ti;
                long id = findID(names, "info");
                if (id >= 0)
                {
                    events.Enqueue(new Event(OWL.Type.TRACKERINFO, (ushort)id, 0, e.time, types[(uint)Type.TRACKERINFO].name, names[(uint)id].name, ti));
                }
            }
            else if (data.StartsWith("table=markers"))
            {
                parseMarkerInfo(markers, data);
                Dictionary<uint, List<uint>> tm = new Dictionary<uint, System.Collections.Generic.List<uint>>();
                foreach (MarkerInfo mi in markers.Values)
                {
                    if (!tm.ContainsKey(mi.tracker_id))
                        tm[mi.tracker_id] = new System.Collections.Generic.List<uint>();
                    tm[mi.tracker_id].Add(mi.id);
                }
                foreach (KeyValuePair<uint, List<uint>> kv in tm)
                {
                    if (trackers.ContainsKey(kv.Key))
                        trackers[kv.Key].marker_ids = kv.Value.ToArray();
                }
                properties["markers"] = markers.Count;
                properties["markerinfo"] = markers.Values.ToArray().ToArray();
                properties["trackerinfo"] = trackers.Values.ToArray();
                long id = findID(names, "info");
                if (id >= 0)
                {
                    events.Enqueue(new Event(Type.MARKERINFO, (ushort)id, 0, e.time, types[(uint)Type.MARKERINFO].name, names[(uint)id].name, properties["markerinfo"]));
                    events.Enqueue(new Event(Type.TRACKERINFO, (ushort)id, 0, e.time, types[(uint)Type.TRACKERINFO].name, names[(uint)id].name, properties["trackerinfo"]));
                }
            }
            else if (data.StartsWith("table=devices") || data.StartsWith("status=devices"))
            {
                if (data.StartsWith("t"))
                    parseDeviceInfo(devices, data);
                else if (data.StartsWith("s"))
                    parseDeviceStatus(devices, data);
                DeviceInfo[] di = devices.Values.Select(x => x).ToArray();
                properties["deviceinfo"] = di;
                long id = findID(names, "info");
                if (id >= 0)
                {
                    events.Enqueue(new Event(Type.DEVICEINFO, (ushort)id, 0, e.time, types[(uint)Type.DEVICEINFO].name, names[(uint)id].name, di));
                }
            }
            else if (data.StartsWith("table=enable"))
            {
                foreach (KeyValuePair<string, string> kv in utils.tomap(data))
                    _options[kv.Key] = kv.Value;
            }
            else if (data.StartsWith("table=pack"))
            {
                List<PackInfo> pi = new List<PackInfo>();
                parsePackInfo(pi, data);
                properties["packinfo"] = pi.ToArray();
                packinfo = pi.ToArray();

                long id = findID(names, "info");
                if (id >= 0)
                {
                    events.Enqueue(new Event(Type.PACKINFO, (ushort)id, 0, e.time, types[(uint)Type.PACKINFO].name, names[(uint)id].name, pi.ToArray()));
                }
            }
            else if (data.StartsWith("filter="))
            {
                parseFilterInfo(_filters, data);
                properties["filterinfo"] = _filters.Values.Where(x => x.period > 0).ToArray();
                properties["filters"] = _filters.Values.Select(x => x.name).ToArray();
                long id = findID(names, "info");
                if (id >= 0)
                {
                    events.Enqueue(new Event(Type.FILTERINFO, (ushort)id, 0, e.time, types[(uint)Type.FILTERINFO].name, names[(uint)id].name, properties["filterinfo"]));
                }
            }
            else if (data.StartsWith("defaultprofile="))
            {
                stringmap d = utils.tomap(data);
                properties["defaultprofile"] = d["defaultprofile"];
            }
            else if (data.StartsWith("profiles.json="))
            {
                properties["profiles.json"] = System.Uri.UnescapeDataString(data);
            }
            else if (data.StartsWith("profiles="))
            {
                properties["profiles"] = utils.splitopts(data)[0].Value;
            }
            else
            {
                foreach (KeyValuePair<string, string> kv in utils.tomap(data))
                {
                    update_property(kv.Key, kv.Value);
                    // deprecated
                    if (kv.Key == "streaming") toggle_broadcast(property<int>("streaming"));
                }
            }
        }

        //
        protected void toggle_broadcast(int v)
        {
            if (v == 3 && sockets.bcast == null)
            {
                sockets.bcast = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                sockets.bcast.EnableBroadcast = true;
                sockets.bcast.ExclusiveAddressUse = false;
                sockets.bcast.SendBufferSize = 64 * 1024;
                sockets.bcast.ReceiveBufferSize = ReceiveBufferSize;
                sockets.bcast.Bind(new IPEndPoint(IPAddress.Any, 8500 + port_offset));
                sockets.bcast.Blocking = false;
            }
            else if (v != 3 && sockets.bcast != null)
            {
                sockets.bcast.Close();
                sockets.bcast = null;
            }
        }

        //
        protected void handle_byte(Event e)
        {
            // don't process raw data, user's job.
            if (e.name == "raw") return;

            string data = utils.UTF8(e.data as byte[]);
            log(e.type_name + " " + e.name + " " + data);
            if (e.name == "options")
            {
                log("options recieved: " + data);
                foreach (KeyValuePair<string, string> kv in utils.tomap(data))
                {
                    _options[kv.Key] = kv.Value;
                }
                List<string> opts = new List<string>();
                foreach (KeyValuePair<string, string> kv in _options)
                {
                    opts.Add(string.Format("{0}={1}", kv.Key, kv.Value));
                }
                properties["options"] = string.Join(" ", opts.ToArray());
            }
            else if (e.name == "initialize" || e.name == "done")
            {
                foreach (KeyValuePair<string, string> kv in utils.tomap(data))
                {
                    update_property(kv.Key, kv.Value);
                    if(kv.Key == "streaming")
                    {
                        int v = 0;
                        if(utils.try_parse(kv.Value, out v))
                            toggle_broadcast(v);
                    }
                }
                if (e.name == "initialize" && properties.ContainsKey("initializing"))
                    properties.Remove("initializing");
            }
        }

        //
        protected void handle_float(Event e)
        {
            log(e);
            if (e.name == "systempose")
            {
                properties["systempose"] = e["systempose"] as float[];
            }
            else if (e.name == "scale")
            {
                properties["scale"] = (e["scale"] as float[])[0];
            }
            else if (e.name == "pose")
            {
                properties["pose"] = e["pose"] as float[];
            }
            else if (e.name == "frequency")
            {
                properties["frequency"] = (e["frequency"] as float[])[0];
            }
        }

        //
        protected void handle_int(Event e)
        {
            log(e);
            if(e.name == "streaming")
            {
                properties["streaming"] = (e["streaming"] as int[])[0];
                toggle_broadcast(streaming());
            }
            else if(e.name == "timebase")
            {
                properties["timebase"] = e["timebase"] as int[];
            }
        }

        //
        protected void handle_camera(Event e)
        {
            if(e.name == "cameras")
            {
                properties["cameras"] = e["cameras"];
            }
        }

        //
        protected static void parseType(Dictionary<uint, TypeInfo> table, string str)
        {
            foreach (KeyValuePair<string, string[]> kv in utils.splitopts(str))
            {
                ushort n;
                if (utils.try_parse(kv.Key, out n))
                {
                    TypeInfo t = new TypeInfo();
                    if (kv.Value.Length > 0 && kv.Value[0].Length > 0)
                        t.name = kv.Value[0];
                    if (kv.Value.Length > 1 && kv.Value[1].Length > 0)
                        utils.try_parse(kv.Value[1], out t.flags);
                    if (kv.Value.Length > 2 && kv.Value[2].Length > 0)
                        utils.try_parse(kv.Value[2], out t.mode);
                    table[n] = t;
                }
            }
        }

        //
        protected static void parseTrackerInfo(Dictionary<uint, TrackerInfo> table, string str)
        {
            uint n = 0;
            TrackerInfo t = null;
            string[] tokens = str.Split();
            foreach (string tok in tokens)
                foreach (KeyValuePair<string, string[]> kv in utils.splitopts(tok))
                    if (kv.Key == "id")
                    {
                        if (kv.Value.Length == 4)
                        {
                            uint tr;
                            if (utils.try_parse(kv.Value[0], out n) && utils.try_parse(kv.Value[1], out tr))
                            {
                                if(!table.ContainsKey(n))
                                {
                                    t = new TrackerInfo((uint)tr,
                                                        kv.Value[2],
                                                        kv.Value[3]);
                                    table[n] = t;
                                }
                                else
                                {
                                    t = table[n];
                                    t.id = (uint) tr;
                                    t.type = kv.Value[2];
                                    t.name = kv.Value[3];
                                    t.options = "";
                                }
                            }
                        }
                    }
                    else if (t != null)
                    {
                        t.options = t.options + (t.options.Length > 0 ? " " : "") + tok;
                    }
        }

        //
        protected static void parseMarkerInfo(Dictionary<uint, MarkerInfo> table, string str)
        {
            ushort id = 0;
            MarkerInfo m = null;
            string[] tokens = str.Split();
            foreach (string tok in tokens)
                foreach (KeyValuePair<string, string[]> kv in utils.splitopts(tok))
                    if (kv.Key == "id")
                    {
                        if (kv.Value.Length == 3)
                        {
                            int tr = -1;
                            if (utils.try_parse(kv.Value[0], out id) && utils.try_parse(kv.Value[1], out tr))
                            {
                                if(!table.ContainsKey(id))
                                {
                                    m = new MarkerInfo(id,
                                                       (uint)tr,
                                                       kv.Value[2]);
                                    table[id] = m;
                                }
                                else
                                {
                                    m = table[id];
                                    m.tracker_id = (uint) tr;
                                    m.name = kv.Value[2];
                                    m.options = "";
                                }
                            }
                        }
                    }
                    else if (m != null)
                    {
                        m.options = m.options + (m.options.Length > 0 ? " " : "") + tok;
                    }
        }

        //
        protected static void parseDeviceInfo(Dictionary<ulong, DeviceInfo> table, string str)
        {
            DeviceInfo d = null;
            string[] tokens = str.Split();
            foreach (string tok in tokens)
            {
                //TODO test
                if (tok == "status=devices")
                {
                    parseDeviceStatus(table, str.Substring(str.IndexOf("status=devices")));
                    break;
                }

                foreach (KeyValuePair<string, string[]> kv in utils.splitopts(tok))
                    if (kv.Key == "hwid")
                    {
                        if (kv.Value.Length == 4)
                        {
                            ulong n, hwid;
                            if (utils.try_parse(kv.Value[0], out n) && utils.try_parse(kv.Value[1], out hwid))
                            {
                                if(!table.ContainsKey(n))
                                {
                                    d = new DeviceInfo(hwid, 0, kv.Value[2], kv.Value[3]);
                                    table[n] = d;
                                }
                                else
                                {
                                    d = table[n];
                                    d.hw_id = hwid;
                                    d.type = kv.Value[2];
                                    d.name = kv.Value[3];
                                    d.options = "";
                                    // do not clear status string
                                }
                            }
                            else if(kv.Value[1] == "-1" && table.ContainsKey(n)) table.Remove(n);
                        }
                    }
                    else if (d != null)
                    {
                        d.options = d.options + (d.options.Length > 0 ? " " : "") + tok;
                    }
            }
        }

        //
        protected static void parseDeviceStatus(Dictionary<ulong, DeviceInfo> table, string str)
        {
            DeviceInfo d = null;
            string[] tokens = str.Split();
            foreach (string tok in tokens)
            {
                foreach (KeyValuePair<string, string[]> kv in utils.splitopts(tok))
                {
                    if (kv.Key == "hwid")
                    {
                        if (kv.Value.Length == 2)
                        {
                            ulong hwid;
                            if (utils.try_parse(kv.Value[0], out hwid) && table.ContainsKey(hwid))
                            {
                                d = table[hwid];
                                utils.try_parse(kv.Value[1], out d.time);
                                d.status = "";
                            }
                        }
                    }
                    else if (d != null)
                    {
                        d.status = d.status + (d.status.Length > 0 ? " " : "") + tok;
                    }
                }
            }
        }

        //
        protected static void parseFilterInfo(Dictionary<string, FilterInfo> table, string str)
        {
            FilterInfo f = new FilterInfo();
            string[] tokens = str.Split();
            foreach (string tok in tokens)
            {
                foreach (KeyValuePair<string, string[]> kv in utils.splitopts(tok))
                {
                    if (kv.Key == "filter" && kv.Value.Length > 0)
                    {
                        f = new FilterInfo();
                        f.name = kv.Value[0];
                        table[f.name] = f;
                    }
                    else if (kv.Key == "period" && kv.Value.Length > 0 && f != null)
                    {
                        utils.try_parse(kv.Value[0], out f.period);
                    }
                    else
                    {
                        f.options = f.options + (f.options.Length > 0 ? " " : "") + tok;
                    }
                }
            }
        }

        //
        protected static void parsePackInfo(List<PackInfo> packinfo, string str)
        {
            string [] tokens = str.Split(new string [] {"type=", }, System.StringSplitOptions.None);
            if(tokens[0].Trim() != "table=pack") throw new OWLError("pack parse error");
            packinfo.Clear();
            for(int i = 1; i < tokens.Length; i++)
            {
                try
                {
                    string [] a = tokens[i].Split(null, 2);
                    string [] b = a[0].Split(new char [] {',',}, 4);
                    PackInfo p = new PackInfo((Type) utils.parse_number<ushort>(b[0]),
                                              b[2],
                                              a.Length > 1 ? a[1] : "",
                                              b[3]);
                    p.id = utils.parse_number<ushort>(b[1]);
                    packinfo.Add(p);
                    utils.log(p);
                }
                catch(System.Exception)
                {
                    utils.log("error parsing token: " + tokens[i]);
                    packinfo.Clear();
                    throw;
                }
            }
        }

        //
        protected delegate bool wait_delegate();

        //
        protected bool wait(long timeout_usec, wait_delegate func)
        {
            if (timeout_usec == 0) return func();
            Timer t = new Timer();
            t.Start();
            do
            {
                if (func()) return true;
            } while (t.ElapsedMicroseconds < timeout_usec);
            return false;
        }
    }

    //
    public struct Stats
    {
        public int udp_packet_count;
        public int udp_bcast_packet_count;
        public int tcp_packet_count;
        public long debug;
    }

    //
    class Sockets
    {
        protected Socket _tcp = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        protected Socket _udp = null;
        protected Socket _bcast = null;
        protected List<Socket> all_sockets = new List<Socket>();

        protected void refresh()
        {
            all_sockets = new List<Socket>(new Socket[]{_tcp, _udp, _bcast}.Where(x => x != null));
        }

        public Socket tcp
        {
            set { _tcp = value; refresh(); }
            get { return _tcp; }
        }

        public Socket udp
        {
            set { _udp = value; refresh(); }
            get { return _udp; }
        }

        public Socket bcast
        {
            set { _bcast = value; refresh(); }
            get { return _bcast; }
        }

        public List<Socket> all
        {
            get { return all_sockets.ToList(); }
        }
    }

    //
    public class Scan
    {
        public int Port = 8998;
        protected UdpClient udp;

        public bool send(string message)
        {
            udp = new UdpClient();
            udp.EnableBroadcast = true;
            udp.ExclusiveAddressUse = false;
            byte[] bytes = System.Text.Encoding.ASCII.GetBytes(message + '\0');
            try
            {
                if (udp.Send(bytes, bytes.Length, "255.255.255.255", Port) < bytes.Length)
                    return false;
            }
            catch (System.Exception e)
            {
                utils.log(e);
                return false;
            }
            return true;
        }

        //
        public string[] listen(int timeout_usec = 1000000)
        {
            List<string> ret = new List<string>();
            if (udp == null) return new string[0];

            try
            {
                while (true)
                {
                    // wait for incoming packets
                    if (udp.Client.Poll(timeout_usec, SelectMode.SelectRead))
                    {
                        while (udp.Available > 0)
                        {
                            IPEndPoint remote = new IPEndPoint(0xffffffff, Port);
                            byte[] bytes = udp.Receive(ref remote);
                            ret.Add(string.Format("ip={0} {1}",
                                                  remote.Address.ToString(),
                                                  System.Text.Encoding.UTF8.GetString(bytes)));
                            // do not wait for much longer after first packet arrives
                            timeout_usec = 10000;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
            catch (System.Exception e)
            {
                utils.log(e);
            }
            return ret.ToArray();
        }

    }

    //
    class Protocol
    {
        //
        [StructLayout(LayoutKind.Sequential)]
        public struct Header
        {
            public ushort id;
            public byte type;
            public byte cksum; // header only
            public uint size; // payload only
            public long time;

            //
            public byte sum()
            {
                byte sum = 0;
                for(int i = 0; i < sizeof(ushort); i++) sum += getbyte(id, i);
                sum += type;
                sum += cksum;
                for(int i = 0; i < sizeof(uint); i++) sum += getbyte(size, i);
                return (byte) sum;
            }

            //
            public bool valid()
            {
                return sum() == 0;
            }

            //
            public void write(System.IO.BinaryWriter bw)
            {
                cksum = 0;
                cksum = (byte) -sum();
                bw.Write(id);
                bw.Write(type);
                bw.Write(cksum);
                bw.Write(size);
                bw.Write(time);
            }
        }

        protected byte[] inbuffer;
        protected byte[] outbuffer;
        protected Header header;
        protected int hdr_size = 0;

        protected System.IO.MemoryStream ims;
        protected System.IO.MemoryStream oms;

        protected System.IO.BinaryReader br;
        protected System.IO.BinaryWriter bw;

        //
        public Protocol(int recv_buffer_size)
        {
            inbuffer = new byte[recv_buffer_size];
            outbuffer = new byte[64 * 1024];
            hdr_size = Marshal.SizeOf(header);
            ims = new System.IO.MemoryStream(inbuffer);
            oms = new System.IO.MemoryStream(outbuffer);
            br = new System.IO.BinaryReader(ims);
            bw = new System.IO.BinaryWriter(oms);
        }

        //
        public bool send(Socket socket, ushort id, byte type, byte[] data)
        {
            oms.Position = 0;

            header.id = id;
            header.type = type;
            header.size = (uint)data.Length;
            header.time = 0;

            header.write(bw);

            return socket.Send(outbuffer, 0, hdr_size, SocketFlags.None) == hdr_size && socket.Send(data) == data.Length;
        }

        //
        protected Event read_event()
        {
            long startpos = ims.Position;
            read_header();
            if(!header.valid())
            {
                utils.log("recv: invalid checksum!");
                return null;
            }
            Event evt = new Event((Type)header.type, header.id, 0, header.time, "", "");
            switch ((Type)evt.type_id)
            {
                case Type.MARKER:
                    evt.data = read_markers();
                    break;
                case Type.RIGID:
                    evt.data = read_rigids();
                    break;
                case Type.PLANE:
                    evt.data = read_planes();
                    break;
                case Type.INPUT:
                    evt.data = read_inputs();
                    break;
                case Type.BYTE:
                    evt.data = read_bytes();
                    break;
                case Type.ERROR:
                    evt.data = read_bytes();
                    break;
                case Type.INT:
                    evt.data = read_ints();
                    break;
                case Type.FLOAT:
                    evt.data = read_floats();
                    break;
                case Type.CAMERA:
                    evt.data = read_cameras();
                    break;
                case Type.PEAK:
                    evt.data = read_peaks();
                    break;
                // These are parsed later down the pipeline
                //case Type.MARKERINFO:
                //    break;
                //case Type.TRACKERINFO:
                //    break;
                //case Type.FILTERINFO:
                //    break;
                //case Type.DEVICEINFO:
                //    break;
                //case Type.PACKINFO:
                //    break;
                //case Type.DEVICEINFO:
                //    break;
                //case Type.EVENT:
                //    break;
                default:
                    break;
            }
            // position at start of next subpacket
            ims.Position = startpos + hdr_size + header.size;
            return evt;
        }

        //
        public Event [] recv(Socket socket)
        {
            ims.Position = 0;
            int nbytes = read_packet(socket);
            if(nbytes <= 0) return null;

            List<Event> events = new List<Event>();
            ims.Position = 0;
            while(ims.Position < nbytes)
            {
                Event evt = read_event();
                if(evt == null) break;
                events.Add(evt);
            }
            return events.ToArray();
        }

        //
        protected virtual int read_packet(Socket socket)
        {
            // do we have enough bytes?
            if (socket.Available < hdr_size)
                return 0;

            // read header
            socket.Receive(inbuffer, hdr_size, SocketFlags.Peek);

            // parse header
            read_header();

            // read rest of packet
            int pkt_size = hdr_size + (int)header.size;
            if(pkt_size > socket.ReceiveBufferSize)
                throw new OWLError("insufficient buffer size: " + pkt_size + " " + socket.ReceiveBufferSize);
            if (socket.Available < pkt_size)
                return 0;
            return socket.Receive(inbuffer, 0, (int)pkt_size, SocketFlags.None);
        }

        //
        protected void read_header()
        {
            header.id = br.ReadUInt16();
            header.type = br.ReadByte();
            header.cksum = br.ReadByte();
            header.size = br.ReadUInt32();
            header.time = br.ReadInt64();
        }

        //
        protected Camera[] read_cameras()
        {
            int sz = 40;
            int n = (int)header.size / sz;
            Camera[] cameras = new Camera[n];
            for (int i = 0; i < n; i++)
            {
                Camera c = new Camera();
                cameras[i] = c;
                c.id = br.ReadUInt32();
                c.flags = br.ReadUInt32();
                for (int j = 0; j < c.pose.Length; j++)
                    c.pose[j] = br.ReadSingle();
                c.cond = br.ReadSingle();
            }
            return cameras;
        }

        //
        protected Peak[] read_peaks()
        {
            int sz = 32;
            int n = (int)header.size / sz;
            Peak[] peaks = new Peak[n];
            for (int i = 0; i < n; i++)
            {
                Peak p = new Peak();
                peaks[i] = p;
                p.id = br.ReadUInt32();
                p.flags = br.ReadUInt32();
                p.time = br.ReadInt64();
                p.camera = br.ReadUInt16();
                p.detector = br.ReadUInt16();
                p.width = br.ReadUInt32();
                p.pos = br.ReadSingle();
                p.amp = br.ReadSingle();
            }
            return peaks;
        }

        //
        protected Plane[] read_planes()
        {
            int sz = 40;
            int n = (int)header.size / sz;
            Plane[] planes = new Plane[n];
            for (int i = 0; i < n; i++)
            {
                Plane p = new Plane();
                planes[i] = p;
                p.id = br.ReadUInt32();
                p.flags = br.ReadUInt32();
                p.time = br.ReadInt64();
                p.camera = br.ReadUInt16();
                p.detector = br.ReadUInt16();
                for (int j = 0; j < p.plane.Length; j++)
                    p.plane[j] = br.ReadSingle();
                p.offset = br.ReadSingle();
            }
            return planes;
        }

        //
        protected Input[] read_inputs()
        {
            br.ReadUInt32(); //count
            uint o = 4;
            List<Input> inputs = new List<Input>();
            while (o < header.size)
            {
                Input inp = new Input();
                inp.hw_id = br.ReadUInt64();
                inp.flags = br.ReadUInt64();
                inp.time = br.ReadInt64();
                uint size = br.ReadUInt32();
                o += 28;
                inp.data = br.ReadBytes((int)size);
                o += size;
                inputs.Add(inp);
            }
            return inputs.ToArray();
        }

        //
        protected Marker[] read_markers()
        {
            int sz = 32;
            int n = (int)header.size / sz;
            Marker[] markers = new Marker[n];
            for (int i = 0; i < n; i++)
            {
                Marker m = new Marker();
                markers[i] = m;
                m.id = br.ReadUInt32();
                m.flags = br.ReadUInt32();
                m.time = br.ReadInt64();
                m.x = br.ReadSingle();
                m.y = br.ReadSingle();
                m.z = br.ReadSingle();
                m.cond = br.ReadSingle();
            }
            return markers;
        }

        //
        protected Rigid[] read_rigids()
        {
            int sz = 48;
            int n = (int)header.size / sz;
            Rigid[] rigids = new Rigid[n];
            for (int i = 0; i < n; i++)
            {
                Rigid r = new Rigid();
                rigids[i] = r;
                r.id = br.ReadUInt32();
                r.flags = br.ReadUInt32();
                r.time = br.ReadInt64();
                for (int j = 0; j < r.pose.Length; j++)
                    r.pose[j] = br.ReadSingle();
                r.cond = br.ReadSingle();
            }
            return rigids;
        }

        //
        protected float[] read_floats()
        {
            int n = (int)header.size / sizeof(float);
            float[] floats = new float[n];
            for (int i = 0; i < n; i++) floats[i] = br.ReadSingle();
            return floats;
        }

        //
        protected int[] read_ints()
        {
            int n = (int)header.size / sizeof(int);
            int[] ints = new int[n];
            for (int i = 0; i < n; i++)
                ints[i] = br.ReadInt32();
            return ints;
        }

        //
        protected byte[] read_bytes()
        {
            return br.ReadBytes((int)header.size);
        }

        //
        public static byte getbyte(ushort v, int b)
        {
            return (byte)(0xff & (v >> (b * 8)));
        }

        //
        public static byte getbyte(int v, int b)
        {
            return (byte)(0xff & (v >> (b * 8)));
        }

        //
        public static byte getbyte(uint v, int b)
        {
            return (byte)(0xff & (v >> (b * 8)));
        }
    }

    //
    class ProtocolUdp : Protocol
    {
        //
        public ProtocolUdp(int recv_buffer_size) : base(recv_buffer_size) {}

        //
        protected override int read_packet(Socket socket)
        {
            if (!socket.Poll(0, SelectMode.SelectRead)) return 0;

            int bytes = socket.Receive(inbuffer);
            // do we have a packet? drop if not
            if (bytes < hdr_size) return 0;

            return bytes;
        }
    }

    //
    public class utils
    {
        //
        public static void log(object msg)
        {
            if (msg == null) msg = "(null)";
#if UNITY_EDITOR
            UnityEngine.Debug.Log(msg);
#else
            System.Console.WriteLine(msg.ToString());
#endif
        }

        //
        public static T parse_number<T>(string str)
        {
            object obj = null;
            string s = str.Trim();
            int nbase = 10;
            if (s.Length <= 0) throw new System.FormatException();

            if (s.StartsWith("0x")) nbase = 16;
            if (typeof(T) == typeof(ushort))
                obj = System.Convert.ToUInt16(s, nbase);
            else if (typeof(T) == typeof(short))
                obj = System.Convert.ToInt16(s, nbase);
            else if (typeof(T) == typeof(uint))
                obj = System.Convert.ToUInt32(s, nbase);
            else if (typeof(T) == typeof(int))
                obj = System.Convert.ToInt32(s, nbase);
            else if (typeof(T) == typeof(ulong))
                obj = System.Convert.ToUInt64(s, nbase);
            else if (typeof(T) == typeof(long))
                obj = System.Convert.ToInt64(s, nbase);
            else if (typeof(T) == typeof(float))
                obj = System.Convert.ToSingle(s);
            else if (typeof(T) == typeof(Type))
                obj = System.Convert.ToUInt16(s, nbase);
            else throw new OWLError("unsupported conversion type");

            return (T)System.Convert.ChangeType(obj, typeof(T));
        }

        //
        public static bool try_parse<T>(string str, out T o)
        {
            o = default(T);
            try
            {
                o = parse_number<T>(str);
            }
            catch (System.OverflowException)
            {
                utils.log("overflow exception while parsing token: " + str);
                return false;
            }
            catch (System.FormatException)
            {
                return false;
            }
            return true;
        }

        //
        public static int get<T>(string value, out T[] v)
        {
            List<T> _v = new List<T>();
            string[] tokens = value.Split(',');
            foreach (string tok in tokens)
                _v.Add(parse_number<T>(tok));
            v = _v.ToArray();
            return _v.Count;
        }

        //
        public static stringmap tomap(string str)
        {
            stringmap ret = new stringmap();
            string[] KV = str.Split();
            foreach (string kv in KV)
            {
                string[] _kv = kv.Split('=');
                if (_kv.Length != 2)
                    continue;
                ret[_kv[0]] = _kv[1];
            }
            return ret;
        }

        //
        public static KeyValuePair<string, string[]>[] splitopts(string str)
        {
            List<KeyValuePair<string, string[]>> ret = new List<KeyValuePair<string, string[]>>();
            string[] opts = str.Split();
            foreach (string o in opts)
            {
                string[] kv = o.Split('=');
                if (kv.Length < 2)
                    continue;
                KeyValuePair<string, string[]> kv2 = new KeyValuePair<string, string[]>(kv[0], kv[1].Split(','));
                ret.Add(kv2);
            }
            return ret.ToArray();
        }

        //
        public static int GetSocketError(Socket socket)
        {
            return (int)socket.GetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Error);
        }

        //
        public static string UTF8(byte [] bytes)
        {
            return System.Text.Encoding.UTF8.GetString(bytes);
        }

        //
        protected enum JSON_STATE
        {
            EXPECT_OBJ, EXPECT_KEY, EXPECT_COLON, PARSING_VALUE, PARSING_VALUE_NUMBER, EXPECT_COMMA_OR_END
        }

        //
        public static Dictionary<string, object> ParseJSON(string text)
        {
            int e = 0;
            return ParseJSON(text, 0, ref e);
        }

        //
        protected static Dictionary<string, object> ParseJSON(string text, int start, ref int end)
        {
            Dictionary<string, object> o = new Dictionary<string, object>();
            JSON_STATE state = JSON_STATE.EXPECT_OBJ;
            string k = null;
            object v = null;
            for(int i = start; i < text.Length; i++)
            {
                char c = text[i];
                switch(state)
                {
                    case JSON_STATE.EXPECT_OBJ:
                        if(System.Char.IsWhiteSpace(c)) continue;
                        else if(c == '{') state = JSON_STATE.EXPECT_KEY;
                        else throw new System.Exception("JSON parsing error");
                        break;
                    case JSON_STATE.EXPECT_KEY:
                        if(System.Char.IsWhiteSpace(c)) continue;
                        else if(c == '"')
                        {
                            k = ParseJSONString(text, i, ref i);
                            state = JSON_STATE.EXPECT_COLON;
                        }
                        else if(c == '}') // not strict JSON
                        {
                            end = i;
                            return o;
                        }
                        else throw new System.Exception("JSON parsing error");
                        break;
                    case JSON_STATE.EXPECT_COLON:
                        if(System.Char.IsWhiteSpace(c)) continue;
                        else if(c == ':')
                        {
                            v = ParseJSONValue(text, i+1, ref i);
                            o[k] = v;
                            state = JSON_STATE.EXPECT_COMMA_OR_END;
                        }
                        else throw new System.Exception("JSON parsing error");
                        break;
                    case JSON_STATE.EXPECT_COMMA_OR_END:
                        if(System.Char.IsWhiteSpace(c)) continue;
                        else if(c == ',') state = JSON_STATE.EXPECT_KEY;
                        else if(c == '}')
                        {
                            end = i;
                            return o;
                        }
                        break;
                }
            }
            throw new System.Exception("JSON parsing error");
        }

        //
        protected static string ParseJSONString(string text, int start, ref int end)
        {
            if(text[start] != '"') throw new System.Exception("JSON parsing error");
            start += 1;
            for(int i = start; i < text.Length; i++)
            {
                char c = text[i];
                if(c == '"')
                {
                    end = i;
                    return text.Substring(start, i - start);
                }
                if(c == '\\' || System.Char.IsControl(c))
                    throw new System.Exception("JSON parsing error");
            }
            throw new System.Exception("JSON parsing error");
        }

        //
        protected static object ParseJSONValue(string text, int start, ref int end)
        {
            JSON_STATE state = JSON_STATE.PARSING_VALUE;
            int index = -1;
            for(int i = start; i < text.Length; i++)
            {
                char c = text[i];
                switch(state)
                {
                    case JSON_STATE.PARSING_VALUE:
                        if(System.Char.IsWhiteSpace(c)) continue;
                        switch(c)
                        {
                            case '"': return ParseJSONString(text, i, ref end);
                            case '[': return ParseJSONArray(text, i, ref end);
                            case '{': return ParseJSON(text, i, ref end);
                            case 't':
                                if(text.Substring(i, 4) != "true")
                                    throw new System.Exception("JSON parsing error");
                                end = i + 4;
                                return true;
                            case 'f':
                                if(text.Substring(i, 5) != "false")
                                    throw new System.Exception("JSON parsing error");
                                end = i + 5;
                                return false;
                            case 'n':
                                if(c == 'n' && text.Substring(i, 4) != "null")
                                    throw new System.Exception("JSON parsing error");
                                end = i + 4;
                                return null;
                            default:
                                state = JSON_STATE.PARSING_VALUE_NUMBER;
                                index = i;
                                break;
                        }
                        break;
                    case JSON_STATE.PARSING_VALUE_NUMBER:
                        if(System.Char.IsDigit(c) || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-')
                            continue;
                        end = i-1;
                        string n = text.Substring(index, i - index);
                        int l = 0;
                        float f = 0;
                        if(System.Int32.TryParse(n, out l)) return l;
                        if(System.Single.TryParse(n, out f)) return f;
                        throw new System.Exception("JSON parsing error");
                }
            }
            throw new System.Exception("JSON parsing error");
        }

        //
        protected static List<object> ParseJSONArray(string text, int start, ref int end)
        {
            JSON_STATE state = JSON_STATE.PARSING_VALUE;
            List<object> list = new List<object>();
            if(text[start] != '[') throw new System.Exception("JSON parsing error");
            for(int i = start+1; i < text.Length; i++)
            {
                char c = text[i];
                switch(state)
                {
                    case JSON_STATE.PARSING_VALUE:
                        if(System.Char.IsWhiteSpace(c)) continue;
                        else if(c == ']')
                        {
                            end = i;
                            return list;
                        }
                        object v = ParseJSONValue(text, i, ref i);
                        list.Add(v);
                        state = JSON_STATE.EXPECT_COMMA_OR_END;
                        break;
                    case JSON_STATE.EXPECT_COMMA_OR_END:
                        if(System.Char.IsWhiteSpace(c)) continue;
                        else if(c == ',') state = JSON_STATE.PARSING_VALUE;
                        else if(c == ']')
                        {
                            end = i;
                            return list;
                        }
                        else throw new System.Exception("JSON parsing error");
                        break;
                }
            }
            throw new System.Exception("JSON parsing error");
        }
    }

    //
    class Timer : System.Diagnostics.Stopwatch
    {
        protected long nsecs_per_tick;
        protected double usecs_per_tick;

        //
        public Timer() : base()
        {
            nsecs_per_tick = (long)1E9 / Frequency;
            usecs_per_tick = 1E6 / Frequency;
            if (nsecs_per_tick == 0) nsecs_per_tick = 1; // hack.
        }

        //
        public long ElapsedNanoseconds
        {
            get
            {
                return ElapsedTicks * nsecs_per_tick;
            }
        }

        //
        public long ElapsedMicroseconds
        {
            get
            {
                return (long) (ElapsedTicks * usecs_per_tick);
            }
        }
    }
}
