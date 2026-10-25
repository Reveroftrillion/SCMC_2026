import sys, UnityPy
env=UnityPy.load(sys.argv[1])
print("########## TextAsset 목록 ##########")
for o in env.objects:
    if o.type.name!="TextAsset": continue
    try:
        d=o.read(); n=getattr(d,'m_Name',None) or getattr(d,'name','?')
        raw=getattr(d,'m_Script',None) or getattr(d,'script',b'')
        if isinstance(raw,str): raw=raw.encode('utf8','replace')
        print(f"  {n}   ({len(raw)} bytes)")
    except Exception as e: print("  err",e)
print("\n########## MonoScript 클래스명 중 식별가능한 것 (제어/자율주행 관련) ##########")
import re
seen=set()
for o in env.objects:
    if o.type.name!="MonoScript": continue
    try:
        d=o.read()
        cn=str(getattr(d,'m_ClassName','')); ns=str(getattr(d,'m_Namespace',''))
        full=f"{ns}.{cn}" if ns else cn
        if full in seen: continue
        seen.add(full)
        if set(cn)<=set('[]'): continue
        if re.search(r'(?i)cruise|auto|decision|control|drive|driv|path|link|mgeo|waypoint|lateral|longitud|steer|accel|brake|hotkey|key', full):
            print("  ", full)
    except Exception: pass
