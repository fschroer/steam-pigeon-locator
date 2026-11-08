import re
import csv, glob, os, math
import os as _os
BASE=_os.environ.get("FLIGHT_DATA", r"G:\My Drive\Rocketry\Flight Data")
def load(path):
    hdr={}; rows=[]; cols=None
    with open(path, newline='') as f:
        for line in f:
            line=line.rstrip('\r\n')
            if cols is None:
                if line.startswith('time_ms'):
                    toks=line.split(',')
                    cols=[]
                    for t in toks:
                        if re.match(r'^-?[0-9.]+$',t): break
                        if t: cols.append(t)
                    continue
                if ':' in line:
                    k,v=line.split(':',1); hdr[k.strip()]=v.strip()
                continue
            p=line.split(',')
            if len(p)<len(cols): continue
            r={}
            for c,v in zip(cols,p):
                try: r[c]=float(v)
                except: r[c]=float('nan')
            rows.append(r)
    return hdr, cols, rows
def files():
    return sorted(glob.glob(BASE+"/*/*.csv"))
