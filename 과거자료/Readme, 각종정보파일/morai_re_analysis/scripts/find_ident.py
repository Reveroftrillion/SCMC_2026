import sys, dnfile
path=sys.argv[1]; needles=[n.lower() for n in sys.argv[2:]]
pe=dnfile.dnPE(path); td=pe.net.mdtables.TypeDef
for i,t in enumerate(td.rows):
    tn=f"{t.TypeNamespace}.{t.TypeName}"
    try: fs=[str(f.row.Name) for f in t.FieldList if f.row]
    except Exception: fs=[]
    try: ms=[str(m.row.Name) for m in t.MethodList if m.row]
    except Exception: ms=[]
    hitf=[x for x in fs if any(n in x.lower() for n in needles)]
    hitm=[x for x in ms if any(n in x.lower() for n in needles)]
    if hitf or hitm:
        print(f"--- {tn}")
        if hitf: print("    FIELDS :", ", ".join(hitf[:20]))
        if hitm: print("    METHODS:", ", ".join(hitm[:20]))
