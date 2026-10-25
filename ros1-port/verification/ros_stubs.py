"""Explicit test-only ROS object/Node doubles. Never import in production.

No discovery, serialization, DDS/TCPROS, scheduling, socket, timer or TF behavior
is simulated. Both original and port get the same controlled boundary, to test
their Python calculation/callback bodies independently of the ROS middleware.
"""
import importlib.util
import sys
import types
from pathlib import Path

ROOT = None
OVERRIDES = {}
PARAM_FILES = []


def module(name, **attrs):
    obj = types.ModuleType(name)
    obj.__dict__.update(attrs)
    obj.__path__ = []
    sys.modules[name] = obj
    if '.' in name:
        parent, child = name.rsplit('.', 1)
        if parent not in sys.modules:
            module(parent)
        setattr(sys.modules[parent], child, obj)
    return obj


class Stamp:
    def __init__(self, sec=0, nanosec=0, secs=None, nsecs=None, **unused):
        self.secs = sec if secs is None else secs
        self.nsecs = nanosec if nsecs is None else nsecs
    sec = property(lambda s: s.secs, lambda s, v: setattr(s, 'secs', v))
    nanosec = property(lambda s: s.nsecs, lambda s, v: setattr(s, 'nsecs', v))
    def to_nsec(self): return int(self.secs)*10**9+int(self.nsecs)
    def to_sec(self): return self.to_nsec()/1e9
    @classmethod
    def now(cls): return cls(secs=100)
    @classmethod
    def from_sec(cls, value): return cls(secs=int(value), nsecs=round((value-int(value))*1e9))


class Time:
    def __init__(self, seconds=0, nanoseconds=0, **unused):
        self.nanoseconds = int(seconds*1e9)+int(nanoseconds)
    @classmethod
    def from_msg(cls, msg): return cls(nanoseconds=msg.to_nsec())
    def to_msg(self): return Stamp(secs=self.nanoseconds//10**9, nsecs=self.nanoseconds%10**9)


class Message:
    _fields = {}
    def __init__(self, *args, **kwargs):
        for name, default in self._fields.items():
            setattr(self, name, default() if callable(default) else default)
        for name, value in zip(self._fields, args): setattr(self, name, value)
        for name, value in kwargs.items(): setattr(self, name, value)


def message(name, fields, **constants):
    return type(name, (Message,), {'_fields': fields, **constants})


class Capture:
    def __init__(self): self.values = []
    def publish(self, msg): self.values.append(msg)
    def get_subscription_count(self): return 0


class Parameter:
    def __init__(self, name, type_=None, value=None): self.name, self.value = name, value


def flatten(values, prefix=''):
    out = {}
    for key, value in values.items():
        full = prefix+key
        if isinstance(value, dict): out.update(flatten(value, full+'.'))
        else: out[full] = value
    return out


def init(args=None, **unused):
    import yaml
    OVERRIDES.clear(); PARAM_FILES.clear()
    args = args or []
    for i, item in enumerate(args):
        if item == '--params-file': PARAM_FILES.append(Path(args[i+1]))
        if item in ('-p', '--param'):
            key, value = args[i+1].split(':=', 1)
            OVERRIDES[key] = yaml.safe_load(value)


class Logger:
    def __getattr__(self, name): return lambda *a, **kw: None


class Node:
    def __init__(self, name, parameter_overrides=None, **unused):
        import yaml
        self._name = name; self._params = {}; self._overrides = {}
        self.publishers = []; self.subscriptions = []; self.timers = []
        for path in PARAM_FILES:
            config = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            for key in ('/**', name, '/'+name):
                self._overrides.update(flatten(config.get(key, {}).get('ros__parameters', {})))
        self._overrides.update(OVERRIDES)
        self._overrides.update({p.name: p.value for p in parameter_overrides or []})
    def declare_parameter(self, name, value=None):
        p = Parameter(name, value=self._overrides.get(name, value)); self._params[name] = p; return p
    def get_parameter(self, name): return self._params[name]
    def set_parameters(self, values):
        for value in values: self._params[value.name] = value
        return [types.SimpleNamespace(successful=True) for _ in values]
    def create_publisher(self, *a, **kw):
        p = Capture(); self.publishers.append(p); return p
    def create_subscription(self, *a, **kw):
        s = types.SimpleNamespace(); self.subscriptions.append(s); return s
    def create_timer(self, *a, **kw):
        t = types.SimpleNamespace(cancel=lambda: None); self.timers.append(t); return t
    def destroy_node(self): pass
    def get_clock(self): return types.SimpleNamespace(now=lambda: Time(seconds=100))
    def get_logger(self): return Logger()
    def get_name(self): return self._name


def install(tree):
    global ROOT
    ROOT = Path(tree)
    Header = message('Header', dict(stamp=Stamp, frame_id=''))
    Point = message('Point', dict(x=0., y=0., z=0.))
    Quaternion = message('Quaternion', dict(x=0., y=0., z=0., w=1. if ROOT.name=='original' else 0.))
    Pose = message('Pose', dict(position=Point, orientation=Quaternion))
    Twist = message('Twist', dict(linear=Point, angular=Point))
    PoseCov = message('PoseWithCovariance', dict(pose=Pose, covariance=lambda: [0.]*36))
    TwistCov = message('TwistWithCovariance', dict(twist=Twist, covariance=lambda: [0.]*36))
    Color = message('ColorRGBA', dict(r=0., g=0., b=0., a=0.))
    Odometry = message('Odometry', dict(header=Header, child_frame_id='', pose=PoseCov, twist=TwistCov))
    PathMsg = message('Path', dict(header=Header, poses=list))
    Marker = message('Marker', dict(header=Header, ns='', id=0, type=0, action=0, pose=Pose,
        scale=Point, color=Color, lifetime=Stamp, frame_locked=False, points=list, colors=list,
        text='', mesh_resource='', mesh_use_embedded_materials=False),
        ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3, LINE_STRIP=4, LINE_LIST=5,
        CUBE_LIST=6, SPHERE_LIST=7, POINTS=8, TEXT_VIEW_FACING=9, MESH_RESOURCE=10, TRIANGLE_LIST=11,
        ADD=0, MODIFY=0, DELETE=2, DELETEALL=3)
    Marker.__slots__=tuple(Marker._fields)
    PointField = message('PointField', dict(name='', offset=0, datatype=0, count=0),
        INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7, FLOAT64=8)
    Cloud = message('PointCloud2', dict(header=Header, height=0, width=0, fields=list,
        is_bigendian=False, point_step=0, row_step=0, data=bytes, is_dense=False))
    Image = message('Image', dict(header=Header, height=0, width=0, encoding='', data=bytes, step=0))
    module('builtin_interfaces.msg', Time=Stamp, Duration=Stamp)
    module('std_msgs.msg', Header=Header, ColorRGBA=Color,
        **{n: message(n, dict(data=d)) for n,d in [('String',''),('Bool',False),('Float64',0.),('Int32',0)]})
    module('geometry_msgs.msg', Point=Point, Vector3=Point, Quaternion=Quaternion, Pose=Pose,
        PoseStamped=message('PoseStamped',dict(header=Header,pose=Pose)),
        PointStamped=message('PointStamped',dict(header=Header,point=Point)),
        TransformStamped=message('TransformStamped',dict(header=Header,child_frame_id='',
            transform=message('Transform',dict(translation=Point,rotation=Quaternion)))))
    module('nav_msgs.msg', Odometry=Odometry, Path=PathMsg)
    module('visualization_msgs.msg', Marker=Marker, MarkerArray=message('MarkerArray',dict(markers=list)))
    module('sensor_msgs.msg', PointCloud2=Cloud, PointField=PointField, Image=Image,
        Imu=message('Imu',dict(header=Header,orientation=Quaternion,angular_velocity=Point,linear_acceleration=Point)),
        NavSatFix=message('NavSatFix',dict(header=Header,latitude=0.,longitude=0.,altitude=0.)))
    custom = {}
    interface_dir = ROOT / ('src/ctrl_interfaces/msg' if (ROOT/'src/ctrl_interfaces/msg').is_dir() else 'src/foscar_interfaces/msg')
    for path in interface_dir.glob('*.msg'):
        fields = {}
        for line in path.read_text(encoding='utf-8').splitlines():
            tokens = line.split('#',1)[0].split()
            if not tokens: continue
            kind, name = tokens[:2]
            fields[name] = list if kind.endswith('[]') else (Header if kind=='std_msgs/Header' else
                ('' if kind=='string' else False if kind=='bool' else 0. if kind.startswith('float') else 0))
        custom[path.stem] = message(path.stem,fields)
    module('ctrl_interfaces.msg', **custom)
    module('foscar_interfaces.msg', **custom)
    class QoS:
        def __init__(self, depth=10, **kw): self.depth=depth; self.__dict__.update(kw)
    class Executor:
        def __init__(self,*a,**kw): pass
        def add_node(self,*a): pass
        def shutdown(self,*a,**kw): pass
    for prefix in ('rclpy','ctrl_ros1'):
        module(prefix,init=init,shutdown=lambda **kw:None,ok=lambda:True,spin=lambda *a,**kw:None)
        module(prefix+('.ctrl_node' if prefix=='ctrl_ros1' else '.node'),Node=Node)
        module(prefix+('.ctrl_parameter' if prefix=='ctrl_ros1' else '.parameter'),Parameter=Parameter)
        module(prefix+('.ctrl_qos' if prefix=='ctrl_ros1' else '.qos'),QoSProfile=QoS,qos_profile_sensor_data=QoS(depth=5),
            DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1,VOLATILE=0))
        module(prefix+('.ctrl_executors' if prefix=='ctrl_ros1' else '.executors'),ExternalShutdownException=type('ExternalShutdownException',(Exception,),{}),
            MultiThreadedExecutor=Executor,SingleThreadedExecutor=Executor)
        module(prefix+('.ctrl_callback_groups' if prefix=='ctrl_ros1' else '.callback_groups'),MutuallyExclusiveCallbackGroup=type('Group',(),{}))
        module(prefix+('.ctrl_time' if prefix=='ctrl_ros1' else '.time'),Time=Time)
        module(prefix+('.ctrl_duration' if prefix=='ctrl_ros1' else '.duration'),Duration=Time)
        module(prefix+('.ctrl_packages' if prefix=='ctrl_ros1' else '.packages'),get_package_share_directory=lambda name:str(ROOT/'src'/name),
            PackageNotFoundError=type('PackageNotFoundError',(Exception,),{}))
    module('rospy',Duration=Stamp,Time=Stamp)
    module('ament_index_python.packages',get_package_share_directory=lambda name:str(ROOT/'src'/name),
        PackageNotFoundError=type('PackageNotFoundError',(Exception,),{}))
    module('tf2_ros',TransformBroadcaster=type('TransformBroadcaster',(),{}))
    # Run the vendored Humble point-cloud representation for both sides. Its
    # ROS1 byte/endian boundary is tested separately in the runtime suite.
    pcpath=Path(__file__).resolve().parents[1]/'src/ctrl_ros1/ctrl_ros1/ctrl_point_cloud2.py'
    spec=importlib.util.spec_from_file_location('ctrl_ros1.ctrl_point_cloud2',pcpath)
    pc=importlib.util.module_from_spec(spec);sys.modules[spec.name]=pc;spec.loader.exec_module(pc)
    sys.modules['ctrl_ros1'].ctrl_point_cloud2=pc
    module('sensor_msgs_py',point_cloud2=pc)
    sys.modules['sensor_msgs_py.point_cloud2']=pc
    adapterpath=pcpath.with_name('ctrl_messages.py')
    if adapterpath.exists():
        spec=importlib.util.spec_from_file_location('ctrl_ros1.ctrl_messages',adapterpath)
        adapter=importlib.util.module_from_spec(spec);sys.modules[spec.name]=adapter;spec.loader.exec_module(adapter)
        sys.modules['ctrl_ros1'].ctrl_messages=adapter
