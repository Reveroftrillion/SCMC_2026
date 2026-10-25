import sys, struct, dnfile
from dncil.cil.body import CilMethodBody
from dncil.cil.body.reader import CilMethodBodyReaderBase

path=sys.argv[1]; NEEDLE=sys.argv[2]; NSS=set(sys.argv[3:])
pe=dnfile.dnPE(path); md=pe.net.mdtables

class R(CilMethodBodyReaderBase):
    def __init__(s,d): s.data=d; s.o=0
    def read(s,n):
        b=s.data[s.o:s.o+n]; s.o+=n; return b
    def tell(s): return s.o
    def seek(s,o): s.o=o; return o

def tokname(tok):
    tok=getattr(tok,'value',tok)
    if not isinstance(tok,int): return None
    tbl=(tok>>24)&0xff; rid=tok&0xffffff
    try:
        if tbl==0x0A: r=md.MemberRef.rows[rid-1]
        elif tbl==0x06: r=md.MethodDef.rows[rid-1]
        elif tbl==0x04: r=md.Field.rows[rid-1]
        elif tbl==0x01: r=md.TypeRef.rows[rid-1]
        elif tbl==0x02: r=md.TypeDef.rows[rid-1]
        elif tbl==0x70:
            return f'"{md.strings_heap if 0 else ""}"'
        else: return None
    except Exception: return None
    nm=str(getattr(r,'Name',getattr(r,'TypeName','')))
    par=''
    try:
        c=r.Class.row
        if c is not None: par=f"{c.TypeNamespace}.{shorten(str(c.TypeName))}::"
    except Exception: pass
    return par+nm
def shorten(s): return '<obf>' if set(s)<=set('[]') else s

def raw_body(rva):
    h=pe.get_data(rva,12)
    if not h: return None
    f=h[0]&3
    if f==2: total=1+(h[0]>>2)
    elif f==3:
        sz=struct.unpack('<I',h[4:8])[0]
        if sz>200000: return None
        total=((struct.unpack('<H',h[0:2])[0]>>12)*4)+sz
    else: return None
    return pe.get_data(rva,total+64)

for ti,t in enumerate(md.TypeDef.rows):
    if str(t.TypeNamespace) not in NSS: continue
    for m in t.MethodList:
        r=m.row
        if not r or not getattr(r,'Rva',0): continue
        code=raw_body(r.Rva)
        if not code: continue
        try: body=CilMethodBody(R(code))
        except Exception: continue
        txt=[]; hit=False
        for ins in body.instructions:
            op=str(ins.opcode); operand=''
            if op.startswith('call') or op=='newobj':
                operand=tokname(ins.operand) or str(ins.operand)
            elif ins.operand is not None:
                if op in ('ldfld','stfld','ldsfld','stsfld','ldflda'):
                    operand=shorten(str(tokname(ins.operand) or ''))
                else: operand=str(ins.operand)
            if NEEDLE in (operand or ''): hit=True
            txt.append(f"    IL_{ins.offset:04X}  {op:<12} {operand}")
        if hit:
            print("="*100)
            print(f"{t.TypeNamespace}.{shorten(str(t.TypeName))} :: {shorten(str(r.Name))}   (IL {len(code)} bytes)")
            print("\n".join(txt))
