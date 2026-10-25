import sys, struct, collections, dnfile
path=sys.argv[1]; nss=set(sys.argv[2:])
pe=dnfile.dnPE(path); md=pe.net.mdtables; td=md.TypeDef
TABLES={6:'MethodDef',10:'MemberRef',0x2B:'MethodSpec'}

def resolve(tok):
    tbl=(tok>>24)&0xff; rid=tok&0xffffff
    try:
        if tbl==10:   r=md.MemberRef.rows[rid-1]
        elif tbl==6:  r=md.MethodDef.rows[rid-1]
        else: return None
    except Exception: return None
    name=str(getattr(r,'Name',''))
    parent=''
    try:
        p=r.Class.row
        if p is not None: parent=f"{p.TypeNamespace}.{p.TypeName}::"
    except Exception: pass
    return parent+name

def body_code(rva):
    hdr=pe.get_data(rva,12)
    if not hdr: return None
    f=hdr[0]&3
    if f==2: return pe.get_data(rva+1, hdr[0]>>2)
    if f==3:
        size=struct.unpack('<I',hdr[4:8])[0]
        if size>200000: return None
        return pe.get_data(rva+((struct.unpack('<H',hdr[0:2])[0]>>12)*4), size)
    return None

calls=collections.Counter(); consts=collections.Counter(); n=0
for t in td.rows:
    if str(t.TypeNamespace) not in nss: continue
    for m in t.MethodList:
        r=m.row
        if not r or not getattr(r,'Rva',0): continue
        code=body_code(r.Rva)
        if not code: continue
        n+=1; i=0; L=len(code)
        while i < L:
            op=code[i]
            if op in (0x28,0x6F,0x73,0x27) and i+5<=L:
                nm=resolve(struct.unpack('<I',code[i+1:i+5])[0])
                if nm: calls[nm]+=1
                i+=5; continue
            if op==0x22 and i+5<=L:
                v=struct.unpack('<f',code[i+1:i+5])[0]
                if -1e6<v<1e6: consts[round(v,6)]+=1
                i+=5; continue
            i+=1
print(f"# 분석한 메서드 본문: {n}개\n")
print("=== 호출되는 수학/벡터 함수 TOP ===")
for k,v in calls.most_common(400):
    if any(s in k for s in ('Mathf::','Math::','Vector3::','Vector2::','Quaternion::','Atan','Sin','Cos','Sqrt','Clamp','Lerp','Abs','Pow','Dot','Cross','Magnitude','Normalize','Angle','Distance','Rad','Deg')):
        print(f"  {v:5d}  {k}")
print("\n=== 자주 등장하는 float 상수 TOP 30 ===")
for k,v in consts.most_common(30): print(f"  {v:5d}  {k}")
