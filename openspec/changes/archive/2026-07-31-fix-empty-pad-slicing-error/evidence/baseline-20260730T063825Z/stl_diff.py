import struct, sys
from collections import Counter

def tris(path):
    d = open(path,'rb').read()
    n = struct.unpack_from('<I', d, 80)[0]
    return Counter(d[84+i*50:84+i*50+50] for i in range(n)), n

a, na = tris(sys.argv[1]); b, nb = tris(sys.argv[2])
only_a = sum((a-b).values()); only_b = sum((b-a).values())
print(f"A tri={na}  B tri={nb}")
print(f"only in A: {only_a}   only in B: {only_b}   共同: {na-only_a}")
print(f"差異比例: {(only_a+only_b)/(na+nb)*100:.4f}%")
