import hashlib

def b_to_mb(b:int) -> float:
    return b/1000000

# Consistant hash function
def uhash(data) -> str:
    h = hashlib.sha256()
    h.update(str(data).encode())
    return h.digest().hex()
