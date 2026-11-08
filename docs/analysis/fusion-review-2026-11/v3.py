import tc
from tc import *
from v2 import ref_tilt, inst
from v1 import F,B
import io,contextlib
for k,f in F.items():
    for gain in [0.02,0.05,0.1,0.2]:
      for mode in ['fw','tilt']:
        r=run(B+f,mode,gain=gain)
        t,e,st=r['t'],r['est'],r['st']; ref=ref_tilt(r['A'],t); ins=inst(r['A'])
        m=(st>=4)&(st<=7); idx=np.where(m)[0]; m&=(t>=t[idx[0]]+10)
        print(f"{k:10s} g={gain:.2f} {mode:4s} med|err|={np.median(np.abs(e[m]-ref[m])):6.1f} p90={np.percentile(np.abs(e[m]-ref[m]),90):6.1f}  inst-vs-ref med={np.median(np.abs(ins[m]-ref[m])):5.1f}")
