"""Practice-only MORAI gRPC adapter, using schemas from the installed simulator.
No GUI operations, vehicle-control RPCs, or network-configuration RPCs.
"""
from pathlib import Path
import grpc
import time
from google.protobuf import descriptor_pb2,descriptor_pool,message_factory

class Morai:
    def __init__(self,address):
        self.channel=grpc.insecure_channel(address)
        self.pool=descriptor_pool.DescriptorPool()
        pending=list(descriptor_pb2.FileDescriptorSet.FromString(Path(__file__).with_name('morai_api.desc').read_bytes()).file)
        while pending:
            remaining=[]
            for d in pending:
                try:self.pool.Add(d)
                except TypeError:remaining.append(d)
            if len(remaining)==len(pending):raise ValueError('unresolved MORAI schema dependencies')
            pending=remaining
    def message(self,name,**kw):return message_factory.GetMessageClass(self.pool.FindMessageTypeByName('morai_sim_api.'+name))(**kw)
    def call(self,service,method,request,response):
        decoder=type(self.message(response)).FromString
        return self.channel.unary_unary('/morai_sim_api.'+service+'/'+method,
            request_serializer=lambda x:x.SerializeToString(),response_deserializer=decoder)(request,timeout=3)
    def checked(self,service,method,request):
        r=self.call(service,method,request,'Result')
        if r.status!=1:raise RuntimeError(f'{method}: {r}')
        return r
    def load(self,name):
        # The API leaves the simulation paused even on successful loads.
        try:self.checked('scenario.Scenario','LoadMoraiScenario',self.message('StringValue',value=name))
        finally:self.checked('simulation.Simulation','Resume',self.message('Empty'))
    def ego(self):
        r=self.call('actor.Actor','GetAllActorsState',self.message('actor.GetAllActorsFilter',vehicle=True),'actor.ActorStates')
        return next(a for a in r.states if a.actor_info.id.value=='Ego'),r.timestamp
    def get_color(self,signal_id):
        q=self.message('infrastructure.GetTrafficLightInfoParam',type=2,value=signal_id)
        r=self.call('infrastructure.Infrastructure','GetTrafficLightInfo',q,'infrastructure.TrafficLightInfo')
        if r.id.value!=signal_id:raise RuntimeError('signal ID mismatch')
        return r.color
    def set_color(self,signal_id,color,siblings=False):
        r=self.message('infrastructure.TrafficLightStateParam',is_impulse=True,set_sibling=siblings)
        r.info.id.value=signal_id;r.info.color=color
        self.checked('infrastructure.Infrastructure','SetTrafficLightState',r)
    def environment(self):
        t=self.call('environment.Environment','GetTime',self.message('Empty'),'environment.SimulationTime')
        w=self.call('environment.Environment','GetWeather',self.message('Empty'),'environment.Weather')
        return {'hour':t.hour,'weather_type':w.type}
    def set_environment(self,hour,weather_type):
        if hour not in (11,13,15) or weather_type not in (0,2):raise ValueError('unsupported collection environment')
        self.checked('environment.Environment','SetTime',self.message('environment.SimulationTime',hour=hour,data_only=False,instantly=True))
        self.checked('environment.Environment','SetWeather',self.message('environment.Weather',type=weather_type,instantly=True))
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            actual=self.environment()
            if actual=={'hour':hour,'weather_type':weather_type}:return actual
            time.sleep(.1)
        raise RuntimeError('environment readback mismatch: '+str(actual))
    def close(self):self.channel.close()
