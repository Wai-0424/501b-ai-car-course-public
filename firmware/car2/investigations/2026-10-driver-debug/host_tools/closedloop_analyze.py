import csv, sys, numpy as np
MAXT=116.0; RPT=2.3275
def exp(p): return 0.0 if p<=0 else (5+0.95*p)/100*MAXT*RPT
ph={}
for r in csv.DictReader(open(sys.argv[1]),delimiter='\t'):
    ph.setdefault(int(float(r['phase'])),[]).append({k:float(v) for k,v in r.items()})
print(' cmd%  exp  |  L: rpm  err%  duty |  R: rpm  err%  duty | L-R%  | peak L/R over steady %')
for p,rs in sorted(ph.items()):
    tgt=rs[0]['tgtL']
    if tgt==0 or p<0: continue
    tail=[r for r in rs if r['t']>=rs[-1]['t']-1.5]
    e=exp(tgt)
    L=np.mean([r['rpmL'] for r in tail]); R=np.mean([r['rpmR'] for r in tail])
    dl=np.mean([r['dutyL'] for r in tail]); dr=np.mean([r['dutyR'] for r in tail])
    pkL=max(r['rpmL'] for r in rs); pkR=max(r['rpmR'] for r in rs)
    diff=(L-R)/((L+R)/2)*100 if (L+R)>1 else float('nan')
    ov=lambda pk,m: (pk-m)/m*100 if m>8 else float('nan')
    print(f'{tgt:4.0f} {e:6.1f} | {L:6.1f} {(L-e)/e*100:+6.1f} {dl:5.0f} | {R:6.1f} {(R-e)/e*100:+6.1f} {dr:5.0f} | {diff:+5.1f} | {ov(pkL,L):+5.1f} / {ov(pkR,R):+5.1f}')
