import sys, UnityPy
env=UnityPy.load(sys.argv[1])
want={'ConstantAccel_0.5','NoSteer','RampSteer_to45deg_in0.2sec','HillTest_ControlInput_Example','Stinger_Accel_Refer','SineWeave_20deg_0.5Hz'}
for o in env.objects:
    if o.type.name!="TextAsset": continue
    d=o.read(); n=str(getattr(d,'m_Name',''))
    if n not in want: continue
    raw=getattr(d,'m_Script',b'')
    if isinstance(raw,str): raw=raw.encode('utf8','replace')
    print("#"*80); print("##",n,f"({len(raw)} bytes)")
    print(raw[:700].decode('utf8','replace'))
    want.discard(n)
