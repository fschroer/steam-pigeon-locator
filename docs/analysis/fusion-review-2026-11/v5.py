from tc import *
from v2 import ref_tilt
from v1 import F,B
for k,f in F.items():
    row=[k]
    for mode,g in [('none',0.02),('fw',0.02),('tilt',0.02),('fw',0.1),('tilt',0.1)]:
        r=run(B+f,mode,gain=g); t,e,st=r['t'],r['est'],r['st']; ref=ref_tilt(r['A'],t)
        m=(st>=4)&(st<=7); idx=np.where(m)[0]; m&=(t>=t[idx[0]]+10); er=np.abs(e[m]-ref[m])
        row.append(f"{np.median(er):.0f}° / {np.percentile(er,90):.0f}°")
    print(' | '.join(row))
