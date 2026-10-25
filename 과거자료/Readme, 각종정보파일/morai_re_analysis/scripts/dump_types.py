import sys, dnfile
path = sys.argv[1]; targets=[t.lower() for t in sys.argv[2:]]
pe = dnfile.dnPE(path)
td = pe.net.mdtables.TypeDef
for i,t in enumerate(td.rows):
    n=str(t.TypeName)
    if not any(x in n.lower() for x in targets): continue
    print("="*95)
    print(f"[row {i+1}] {t.TypeNamespace}.{n}")
    try:
        ex=t.Extends.row
        print("   extends:", f"{ex.TypeNamespace}.{ex.TypeName}" if ex else None)
    except Exception: pass
    try:
        fs=[str(f.row.Name) for f in t.FieldList if f.row]
        print(f"   fields({len(fs)}):"); 
        for k in range(0,len(fs),6): print("      "+", ".join(fs[k:k+6]))
    except Exception as e: print("   fields err",e)
    try:
        ms=[str(m.row.Name) for m in t.MethodList if m.row]
        print(f"   methods({len(ms)}):")
        for k in range(0,len(ms),6): print("      "+", ".join(ms[k:k+6]))
    except Exception as e: print("   methods err",e)
