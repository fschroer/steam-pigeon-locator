from tc import *
F={'Ken_6':'2026_09_04 Pasco WA Sod Blaster/Ken_6_Locator_2026-09-06_133020.csv',
 'Ken132857':'2026_09_04 Pasco WA Sod Blaster/Ken_Locator_2026-09-05_132857.csv',
 'Mike_5':'2026_09_04 Pasco WA Sod Blaster/Mike_5_Locator_2026-09-06_180444.csv',
 'Mike_8':'2026_09_04 Pasco WA Sod Blaster/Mike_8_Locator_2026-09-06_180444.csv'}
from load import BASE as _B; B=_B+'/'
for k,f in F.items():
    r=run(B+f,'none')
    t,lt,e,st=r['t'],r['lt'],r['est'],r['st']
    for lo,hi in [(r['launch'],r['launch']+3),(r['launch']+3,r['launch']+8)]:
        m=(t>=lo)&(t<hi)
        print(k,f"{lo:.1f}-{hi:.1f}s", 'median |replay-logged| %.1f'%np.median(np.abs(e[m]-lt[m])), 'logged %.1f replay %.1f'%(np.median(lt[m]),np.median(e[m])))
    m=st>=4; print(k,'descent median logged %.1f replay %.1f'%(np.median(lt[m]),np.median(e[m])))
