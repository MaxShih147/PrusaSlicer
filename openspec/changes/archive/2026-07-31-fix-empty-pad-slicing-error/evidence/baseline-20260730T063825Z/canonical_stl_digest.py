import hashlib, struct, sys

def canonical_digest(path):
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack_from('<I', data, 80)[0]
    recs = [data[84 + i*50 : 84 + i*50 + 50] for i in range(n)]
    recs.sort()
    h = hashlib.sha256()
    h.update(struct.pack('<I', n))
    for r in recs:
        h.update(r)
    return n, h.hexdigest()

for p in sys.argv[1:]:
    n, d = canonical_digest(p)
    print(f"{d}  tri={n}  {p}")
