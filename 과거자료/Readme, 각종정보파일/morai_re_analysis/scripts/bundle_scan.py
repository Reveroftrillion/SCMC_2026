import sys, collections, UnityPy
env = UnityPy.load(sys.argv[1])
c = collections.Counter()
objs = list(env.objects)
print("총 오브젝트:", len(objs))
for o in objs: c[o.type.name] += 1
for k,v in c.most_common(30): print(f"  {v:7d}  {k}")
