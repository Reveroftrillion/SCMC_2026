import sys, struct, dnfile
from dncil.cil.body import CilMethodBody
from dncil.cil.body.reader import CilMethodBodyReaderBase
path=sys.argv[1]; out=sys.argv[2]; NSS=set(sys.argv[3:])
pe=dnfile.dnPE(path); md=pe.net.mdtables; US=pe.net.user_strings
def sh(s):
    s=str(s); return '<obf%d>'%len(s) if s and set(s)<=set('[]') else s
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
    if tbl==0x70:
        try: return '"%s"'%US.get(rid)
        except Exception: return '"?"'
    try:
        r={0x0A:md.MemberRef,0x06:md.MethodDef,0x04:md.Field,0x01:md.TypeRef,0x02:md.TypeDef}[tbl].rows[rid-1]
    except Exception: return None
    nm=sh(getattr(r,'Name',getattr(r,'TypeName','')))
    par=''
    try:
        c=r.Class.row
        if c is not None: par=f"{c.TypeNamespace}.{sh(c.TypeName)}::"
    except Exception: pass
    return par+nm
def raw(rva):
    h=pe.get_data(rva,12)
    if not h: return None
    f=h[0]&3
    if f==2: tot=1+(h[0]>>2)
    elif f==3:
        sz=struct.unpack('<I',h[4:8])[0]
        if sz>200000: return None
        tot=((struct.unpack('<H',h[0:2])[0]>>12)*4)+sz
    else: return None
    return pe.get_data(rva,tot+64)
f=open(out,'w')
nm_i=0
for t in md.TypeDef.rows:
    if str(t.TypeNamespace) not in NSS: continue
    f.write("\n"+"#"*100+f"\n## TYPE {t.TypeNamespace}.{sh(t.TypeName)}\n")
    try: f.write("## FIELDS: "+", ".join(sh(x.row.Name) for x in t.FieldList if x.row)+"\n")
    except Exception: pass
    for m in t.MethodList:
        r=m.row
        if not r or not getattr(r,'Rva',0): continue
        code=raw(r.Rva)
        if not code: continue
        try: body=CilMethodBody(R(code))
        except Exception: continue
        nm_i+=1
        f.write("\n"+"="*90+f"\nMETHOD #{nm_i}  {sh(r.Name)}\n")
        for ins in body.instructions:
            op=str(ins.opcode); o=ins.operand
            s=''
            if o is not None:
                s=tokname(o)
                if s is None: s=str(o)
            f.write(f"  IL_{ins.offset:04X}  {op:<13} {s}\n")
f.close(); print("methods dumped:",nm_i,"->",out)
