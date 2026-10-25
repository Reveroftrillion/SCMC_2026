import sys, dnfile
path=sys.argv[1]; nss=sys.argv[2:]
pe=dnfile.dnPE(path); td=pe.net.mdtables.TypeDef
def clean(s):
    s=str(s)
    return s if not set(s) <= set('[]') else f'<obf:{len(s)}>'
for i,t in enumerate(td.rows):
    if str(t.TypeNamespace) not in nss: continue
    print("="*95); print(f"[{i+1}] {t.TypeNamespace}.{clean(t.TypeName)}")
    try:
        ex=t.Extends.row
        if ex: print("   extends:", f"{ex.TypeNamespace}.{clean(ex.TypeName)}")
    except Exception: pass
    for label,lst in (("FIELDS",t.FieldList),("METHODS",t.MethodList)):
        try: names=[str(x.row.Name) for x in lst if x.row]
        except Exception: names=[]
        vis=[n for n in names if not set(n) <= set('[]')]
        print(f"   {label} ({len(names)} 총, {len(vis)} 식별가능):")
        for k in range(0,len(vis),5): print("      "+", ".join(vis[k:k+5]))
