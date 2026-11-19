import json,collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'Liberation Serif','font.size':10,'axes.labelsize':10,'legend.fontsize':8.5,'figure.dpi':200,'axes.grid':True,'grid.alpha':.25})
R=[json.loads(l) for l in open('res.jsonl')]+[json.loads(l) for l in open('res6.jsonl')]
def grp(tag,key):
    d=collections.defaultdict(list)
    for r in R:
        if r['tag']==tag:d[key(r)].append(r)
    return d
def ms(rs,f):
    v=np.array([r[f] for r in rs if r[f] is not None],float);return v.mean(),v.std(ddof=1) if len(v)>1 else 0
BL='#1a5fb4';OR='#c64600';GR='#2e7d32';RD='#a51d2d'
out={}
# ---- Fig 2 output vs offered load (E1)
e1=grp('E1',lambda r:r['over']['flow']);qs=sorted(e1)
off=[q+200 for q in qs]
veh=[ms(e1[q],'veh') for q in qs];ppl=[ms(e1[q],'ppl') for q in qs]
fig,axs=plt.subplots(1,2,figsize=(6.5,2.9))
ax=axs[0]
ax.errorbar(off,[v[0] for v in veh],yerr=[v[1] for v in veh],fmt='o-',color=BL,ms=4,lw=1.3,capsize=2,label='Vehicles/h')
ax.plot([1200,4800],[1200,4800],'k:',lw=1);ax.set_xlabel('Offered load (veh/h)');ax.set_ylabel('Vehicles delivered (veh/h)',color=BL)
ax2=ax.twinx();ax2.grid(False);ax2.errorbar(off,[v[0] for v in ppl],yerr=[v[1] for v in ppl],fmt='s--',color=OR,ms=3.5,lw=1,capsize=2);ax2.set_ylabel('People delivered (persons/h)',color=OR)
ax.set_ylim(1000,5200);ax2.set_ylim(1500,7800);ax.set_xlim(1200,4800);ax.text(1300,4500,'1:1 line',fontsize=8)
ax=axs[1]
ug=[ms(e1[q],'qGrow')[0]*1.5 for q in qs]
ax.bar(off,ug,width=250,color=RD,alpha=.8);ax.set_xlabel('Offered load (veh/h)');ax.set_ylabel('Unserved main-road demand (persons/h)')
ax.set_xlim(1000,4800)
fig.tight_layout(pad=.4,w_pad=1);fig.savefig('fig2/g2.png');plt.close()
out['E1']={q:{k:ms(e1[q],k) for k in ('veh','ppl','mainOut','rampOut','trip','qGrow','cong','tetPerKvm','nBD')} for q in qs}
# ---- Fig 3 ramp displacement (E1r)
e1r=grp('E1r',lambda r:r['over']['ramp']);rr=sorted(e1r)
mo=[ms(e1r[r],'mainOut')[0]*1.5 for r in rr];ro=[ms(e1r[r],'rampOut')[0]*1.5 for r in rr];tot=[ms(e1r[r],'ppl')[0] for r in rr]
tsd=[ms(e1r[r],'ppl')[1] for r in rr]
fig,axs=plt.subplots(1,2,figsize=(6.5,2.9))
ax=axs[0]
ax.plot(rr,mo,'o-',color=BL,ms=4,label='Mainline');ax.plot(rr,ro,'s-',color=GR,ms=4,label='Ramp')
ax.errorbar(rr,tot,yerr=tsd,fmt='D-',color='k',ms=4,capsize=2,label='Total')
ax.set_xlabel('Ramp demand (veh/h)');ax.set_ylabel('People delivered (persons/h)');ax.legend(loc='lower left')
ax=axs[1]
ax.plot(rr,[ms(e1r[r],'trip')[0] for r in rr],'o-',color=OR,ms=4);ax.set_ylabel('Mean trip time (s)',color=OR);ax.set_xlabel('Ramp demand (veh/h)')
ax2=ax.twinx();ax2.grid(False);ax2.plot(rr,[ms(e1r[r],'rampBlkPct')[0] for r in rr],'s--',color=RD,ms=4);ax2.set_ylabel('Ramp arrivals turned away (%)',color=RD)
fig.tight_layout(pad=.4,w_pad=1);fig.savefig('fig2/g3.png');plt.close()
out['E1r']={r:{k:ms(e1r[r],k) for k in ('veh','ppl','mainOut','rampOut','trip','rampBlkPct','qGrow','tetPerKvm','cong')} for r in rr}
# ---- lever data
E3=grp('E3',lambda r:(r['over']['rlen'],r['over']['meter']))
E2T=grp('E2T',lambda r:r['over']['headway']);E2b=grp('E2b',lambda r:r['over']['bsafe'])
base=E2T[1.0]  # default config 2800+600
Ls=[200,400,700,1000]
La={L:[r for m in (2,3,4,6) for r in E3[(L,m)]] for L in Ls}
fig,axs=plt.subplots(3,3,figsize=(6.5,6.2),sharex='col')
cols=[('Following gap T (s)',sorted(E2T),E2T),('Lane-change safe braking (ft/s²)',sorted(E2b),E2b),('Acceleration lane length (ft)',Ls,La)]
for j,(lab,keys,dd) in enumerate(cols):
    for i,(f,yl,c) in enumerate((('ppl','Persons/h',BL),('tetPerKvm','TTC < 2 s (veh·s per 1,000 veh-mi)',OR),('hb9PerKvm','Hard braking events\nper 1,000 veh-mi',RD))):
        ax=axs[i,j];m=[ms(dd[k],f) for k in keys]
        ax.errorbar(keys,[a for a,b in m],yerr=[b for a,b in m],fmt='o-',color=c,ms=4,capsize=2,lw=1.2)
        if i==1 and j==0:ax.set_yscale('symlog',linthresh=100)
        if j==0:ax.set_ylabel(yl)
        if i==2:ax.set_xlabel(lab)
        if i==1 and j==0:ax.set_yticks([0,100,1000]);ax.set_yticklabels(['0','100','1,000'])
    axs[0,j].set_ylim(3000,5100) if j>0 else axs[0,j].set_ylim(3000,5100)
    axs[1,j].set_ylim(-20,2000) if j==0 else axs[1,j].set_ylim(0,320)
    axs[2,j].set_ylim(0,460)
for ax,l in zip(axs[0],'abc'):ax.set_title(f'({l})',fontsize=10)
fig.tight_layout(pad=.4,h_pad=.6,w_pad=.5);fig.savefig('fig2/g5.png');plt.close()
def row(rs):return {k:ms(rs,k) for k in ('veh','ppl','tetPerKvm','hb9PerKvm','hb15PerKvm','trip','rampBlkPct','qGrow','mergeTight','nMerge','mergeForced','cong','vmtPerH')}
out['T']={k:row(E2T[k]) for k in sorted(E2T)};out['b']={k:row(E2b[k]) for k in sorted(E2b)}
out['L']={k:row(La[k]) for k in Ls}
E2lc=grp('E2lc',lambda r:1);E3M=grp('E3M',lambda r:r['over']['rampm'])
out['lc_off']=row(E2lc[1]);out['M']={2000:row(E3M[2000]),2500:row(E2T[1.0]),3000:row(E3M[3000])}
out['base']=row(E2T[1.0])
out['meter']={m:row([r for L in Ls for r in E3[(L,m)]]) for m in (2,3,4,6)}
# ---- Fig 6 trade-off scatter
pts=[]
def add(lab,rs,mk,col):pts.append((lab,ms(rs,'ppl')[0],ms(rs,'tetPerKvm')[0],ms(rs,'hb9PerKvm')[0],mk,col))
for T in sorted(E2T):add(f'T={T}',E2T[T],'o',BL)
for b in sorted(E2b):
    if b!=13:add(f'b_safe={b}',E2b[b],'s',OR)
for L in Ls:
    for m in (2,3,4,6):add(f'L={L},m={m}',E3[(L,m)],'^',GR)
add('no lane changes',E2lc[1],'X',RD)
for M in (2000,3000):add(f'M={M}',E3M[M],'D','#6a1b9a')
fig,ax=plt.subplots(figsize=(6.2,3.7))
seen=set()
for lab,y,x,h,mk,col in pts:
    grpn={'o':'Following gap T','s':'Lane-change safe braking','^':'Acceleration lane × meter wait','X':'No lane changes','D':'Merge location'}[mk]
    ax.scatter([x+1],[y],marker=mk,color=col,s=38,alpha=.85,label=None if grpn in seen else grpn);seen.add(grpn)
ax.scatter([ms(E2T[1.0],'tetPerKvm')[0]+1],[ms(E2T[1.0],'ppl')[0]],marker='*',s=170,color='k',zorder=5,label='Default design')
ax.set_xscale('log');ax.set_xlabel('Conflict exposure: time with TTC < 2 s (veh·s per 1,000 veh-mi, log scale)');ax.set_ylabel('People delivered (persons/h)')
for lab,y,x,h,mk,col in pts:
    if lab in('T=0.8','T=1.5','T=2.0','no lane changes','b_safe=6'):ax.annotate(lab,(x+1,y),textcoords='offset points',xytext=(6,-3),fontsize=8)
# pareto: maximize ppl, minimize tet
P=sorted([(x,y) for lab,y,x,h,mk,col in pts]);fr=[];best=-1
for x,y in P:
    if y>best:fr.append((x+1,y));best=y
ax.step([a for a,b in fr],[b for a,b in fr],where='post',color='k',lw=1,ls='--',label='Non-dominated frontier')
ax.legend(loc='lower right',fontsize=8)
fig.tight_layout(pad=.4);fig.savefig('fig2/g6.png');plt.close()
out['pareto']=[(round(x-1,1),round(y)) for x,y in fr]
# ---- Fig 7 incident
b=grp('E5base',lambda r:1)[1];c1=grp('E5',lambda r:r['over']['clear'])
labs=['No crash','Crash, 0.25 h\nclearance','Crash, ≥ 0.5 h\nclearance*'];vals=[ms(b,'ppl'),ms(c1[0.25],'ppl'),ms(c1[0.5],'ppl')]
fig,ax=plt.subplots(figsize=(4.6,2.9))
ax.bar(labs,[v[0] for v in vals],yerr=[v[1] for v in vals],color=[GR,OR,RD],capsize=3,width=.55)
for i,v in enumerate(vals):ax.text(i,v[0]+v[1]+25,f'{v[0]:,.0f}',ha='center',fontsize=9)
ax.set_ylim(2600,3800);ax.set_ylabel('People delivered (persons/h)')
fig.tight_layout(pad=.4);fig.savefig('fig2/g7.png');plt.close()
out['E5']={'base':vals[0],'c25':vals[1],'c50':vals[2],'crashinfo':[r['crashInfo'] for r in R if r['tag']=='E5' and r['over']['clear']==0.5]}
# ---- Fig 8 heatmap
fig,axs=plt.subplots(1,2,figsize=(6.5,2.8))
ms_=[2,3,4,6]
for ax,f,t,cm in ((axs[0],'ppl','Persons/h','YlGn'),(axs[1],'tetPerKvm','TTC < 2 s (veh·s / 1,000 veh-mi)','YlOrRd')):
    Z=np.array([[ms(E3[(L,m)],f)[0] for m in ms_] for L in Ls])
    im=ax.imshow(Z,cmap=cm,aspect='auto',origin='lower')
    ax.set_xticks(range(4));ax.set_xticklabels(ms_);ax.set_yticks(range(4));ax.set_yticklabels(Ls);ax.grid(False)
    nz=(Z-Z.min())/(Z.max()-Z.min()+1e-9)
    for i in range(4):
        for j in range(4):ax.text(j,i,f'{Z[i,j]:,.0f}',ha='center',va='center',fontsize=8.5,color='white' if nz[i,j]>0.7 else 'black')
    ax.set_xlabel('Meter wait (s/car)');ax.set_title(t,fontsize=9.5)
axs[0].set_ylabel('Acceleration lane (ft)')
fig.tight_layout(pad=.4);fig.savefig('fig2/g8.png');plt.close()
# ---- Fig 9 metering optimization (E6)
e6={fl:grp('E6',lambda r,fl=fl:(r['over']['flow'],r['over']['meter'])) for fl in (2800,3200)}
fig,axs=plt.subplots(1,3,figsize=(6.5,2.7))
mt=[2,6,8,12,20]
for fl,col in ((2800,BL),(3200,OR)):
    d=e6[fl]
    m=[ms(d[(fl,x)],'ppl') for x in mt]
    axs[0].errorbar(mt,[a for a,b in m],yerr=[b for a,b in m],fmt='o-',color=col,capsize=2,ms=3.5,label=f'Mainline {fl:,}')
    axs[1].plot(mt,[ms(d[(fl,x)],'rampBlkPct')[0] for x in mt],'o-',color=col,ms=3.5)
    axs[2].plot(mt,[ms(d[(fl,x)],'tetPerKvm')[0] for x in mt],'o-',color=col,ms=3.5)
axs[0].set_ylabel('People delivered (persons/h)');axs[0].legend(loc='lower right',fontsize=7.5)
axs[1].set_ylabel('Ramp arrivals turned away (%)');axs[2].set_ylabel('TTC < 2 s (veh·s / 1,000 veh-mi)')
for a in axs:a.set_xlabel('Meter wait (s/car)');a.set_xticks(mt)
fig.tight_layout(pad=.4,w_pad=.8);fig.savefig('fig2/g9.png');plt.close()
out['E6']={fl:{x:{k:ms(e6[fl][(fl,x)],k) for k in ('veh','ppl','mainOut','rampOut','rampBlkPct','trip','tetPerKvm','hb9PerKvm','rampWait','cong')} for x in mt} for fl in (2800,3200)}
json.dump(out,open('out2.json','w'),default=float,indent=1)
print('done')
