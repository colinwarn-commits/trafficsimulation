import json,collections
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Polygon,Circle
from matplotlib.colors import ListedColormap
import colorsys
plt.rcParams.update({'font.family':'Liberation Serif','font.size':10,'axes.labelsize':10,'legend.fontsize':8.5,'figure.dpi':200,'axes.grid':True,'grid.alpha':.25})
MPH=1.4667;M2F=3.28084
def hsl(h,s,l):
    r,g,b=colorsys.hls_to_rgb(h/360,l,s);return (r,g,b)
cm=ListedColormap([hsl(5+min(1,max(0,(i-5)/45))*195,.8,.52) for i in range(61)])
A=json.load(open('runA.json'));B=json.load(open('runB.json'))

# ---- Fig 1 layout
fig,ax=plt.subplots(figsize=(6.5,3.0))
ax.set_xlim(-60,4080);ax.set_ylim(-480,200);ax.axis('off');ax.grid(False)
road='#3a3d3f'
ax.add_patch(Rectangle((0,0),4000,40,color=road,zorder=1));ax.plot([0,4000],[20,20],color='w',ls=(0,(6,6)),lw=.8,zorder=2)
ax.text(30,30,'Left',color='#ddd',fontsize=6.5,zorder=3,va='center');ax.text(30,9,'Right',color='#ddd',fontsize=6.5,zorder=3,va='center')
ax.plot([2000,2500],[-130,-8],color=road,lw=5,solid_capstyle='butt',zorder=1)
ax.add_patch(Polygon([(2500,-14),(2622,-2),(2622,0),(2500,0)],color=road,zorder=1))
ax.plot([2500,2500],[-40,60],color='k',ls=':',lw=1)
ax.annotate('',xy=(0,115),xytext=(4000,115),arrowprops=dict(arrowstyle='<->',lw=.7))
ax.text(2000,128,'Main road, 2 lanes, 4,000 m',ha='center',fontsize=8.5)
for nm,x in (('A',1500),('D',3300)):
    ax.plot([x,x],[-6,46],color='#e8b92a',lw=3,zorder=4);ax.scatter([x],[75],s=230,color='#e8b92a',zorder=5);ax.text(x,75,nm,ha='center',va='center',fontweight='bold',fontsize=8.5,zorder=6)
ax.text(2530,-60,'Merge, x = 2,500 m\naccel. lane 122 m',ha='left',va='center',fontsize=8)
ax.scatter([2000],[-130],s=230,color='#e8b92a',zorder=5);ax.text(2000,-130,'B',ha='center',va='center',fontweight='bold',fontsize=8.5,zorder=6)
for i in range(7):ax.add_patch(Rectangle((1930-i*60,-139),50,18,color=plt.cm.RdYlGn(.15+i*.1),zorder=3))
ax.scatter([1480],[-130],s=230,color='#e8b92a',zorder=5);ax.text(1480,-130,'C',ha='center',va='center',fontweight='bold',fontsize=8.5,zorder=6)
ax.text(2060,-190,'B: ramp entry and meter',ha='left',fontsize=8);ax.text(1430,-190,'C: ramp queue (n$_{max}$ = 20)',ha='right',fontsize=8)
ax.text(2250,-60,'Ramp 500 m',ha='center',fontsize=8,rotation=14)
ax.text(0,-50,'Origin',fontsize=8);ax.text(4000,-50,'Exit (x = 4,000 m)',ha='right',fontsize=8)
yb=-290
for (a_,b_) in ((0,1500),(1500,2500),(2500,3300),(3300,4000)):
    ax.annotate('',xy=(a_,yb),xytext=(b_,yb),arrowprops=dict(arrowstyle='<->',lw=.7))
    ax.text((a_+b_)/2,yb+18,f'{b_-a_:,} m',ha='center',fontsize=8)
ax.plot([0,4000],[-370,-370],color='k',lw=.8)
for x in range(0,4001,500):
    ax.plot([x,x],[-370,-390],color='k',lw=.8);ax.text(x,-445,f'{x:,}',ha='center',fontsize=8)
ax.text(4080,-370,'x (m)',ha='right',va='bottom',fontsize=8)
fig.tight_layout(pad=.3);fig.savefig('fig/f1.png');plt.close()

# ---- Fig 2 time series
fig,axs=plt.subplots(2,1,figsize=(6.5,4.6),sharex=True)
for ax,d,lab in ((axs[0],A,'(a) Main demand 2,000 veh/h + ramp 200 veh/h'),(axs[1],B,'(b) Main demand 3,000 veh/h + ramp 200 veh/h')):
    r=[x for x in d['rows']]
    t=np.array([x['t'] for x in r])/60
    f=lambda k:np.array([np.nan if x[k] is None else x[k] for x in r])
    ax.plot(t,f('aRu'),lw=1.2,color='#e07b39',label='A, right lane')
    ax.plot(t,f('aLu'),lw=1.2,color='#e07b39',ls='--',label='A, left lane')
    ax.plot(t,f('dRu'),lw=1.2,color='#3b8eea',label='D, right lane')
    ax.axhline(18,color='k',lw=1,ls=':');ax.text(60.5,18.6,'18 mph',ha='right',fontsize=8)
    ax.axvspan(0,10,color='grey',alpha=.15,lw=0)
    for e in d['bd']:
        ax.axvspan(e['t']/60,(e['end'] if e['end'] is not None else d['t'])/60,color='#e5252a',alpha=.12,lw=0)
    ax.set_ylabel('Space-mean speed (mph)');ax.set_title(lab,fontsize=9.5,loc='left');ax.set_ylim(0,36)
axs[1].set_xlabel('Time (min)');axs[0].legend(ncol=3,loc='lower left')
axs[1].set_xlim(0,60)
fig.tight_layout(pad=.4);fig.savefig('fig/f2.png');plt.close()

# ---- Fig 3 x-t
def xt(d,L):
    sf=d['sf'][L];cols=[s['a'] for s in sf];Z=np.array([[np.nan if v is None else v for v in s['row']] for s in sf]).T
    return cols,Z
fig,axs=plt.subplots(1,2,figsize=(6.5,3.6),sharey=True)
for L,ax in enumerate(axs):
    cols,Z=xt(B,L);t0=cols[0]*2/60;t1=(cols[-1]+1)*2/60
    ax.imshow(np.ma.masked_invalid(Z),origin='lower',aspect='auto',cmap=cm,vmin=0,vmax=60,extent=[t0,t1,0,Z.shape[0]*100/M2F],interpolation='nearest')
    for tr in B['trs']:
        if tr['L']!=L:continue
        p=np.array(tr['p']);ax.plot(p[:,0]/60,p[:,1],color='k',alpha=.3,lw=.35)
    for nm,x in (('A',1500),('D',3300)):
        ax.axhline(x,color='k',ls='--',lw=.7);ax.text(t0+.1,x+30,nm,fontsize=8,fontweight='bold')
    if L==0:
        ax.axhline(2500,color='k',ls='--',lw=.7);ax.text(t0+.1,2530,'merge',fontsize=8)
        ax.axhline(2622,color='k',ls='--',lw=.7)
    ax.set_xlim(t0,t1);ax.set_ylim(0,4000);ax.set_xlabel('Time (min)');ax.set_title(['(a) Right (merge) lane','(b) Left lane'][L],fontsize=9.5);ax.grid(False)
axs[0].set_ylabel('Position x (m)')
sm=plt.cm.ScalarMappable(cmap=cm,norm=plt.Normalize(0,60));cb=fig.colorbar(sm,ax=axs,fraction=.04,pad=.02);cb.set_label('Mean speed (mph)')
fig.savefig('fig/f3.png',bbox_inches='tight');plt.close()

# ---- Fig 4 q-k
fig,ax=plt.subplots(figsize=(6.2,3.5))
eq=np.array(B['eq']);ax.plot(eq[:,0],eq[:,1],color='#e8b92a',lw=2.5,label='IDM equilibrium')
cap=eq[:,1].max();kc=eq[eq[:,1].argmax(),0]
for d,m_ in ((A,'o'),(B,'s')):pass
def pts(k_q,k_k,k_u=None):
    out=[]
    for d in (A,B):
        for r in d['rows']:
            if r['t']<=600:continue
            out.append((r[k_k],r[k_q]))
    return np.array(out)
for kq,kk,c,fill,lab in (('aRq','aRk','#e07b39',True,'A, right lane'),('aLq','aLk','#e07b39',False,'A, left lane'),('dRq','dRk','#3b8eea',True,'D, right lane'),('dLq','dLk','#3b8eea',False,'D, left lane')):
    p=pts(kq,kk)
    if fill:ax.scatter(p[:,0],p[:,1],s=9,color=c,alpha=.6,label=lab,lw=0)
    else:ax.scatter(p[:,0],p[:,1],s=12,facecolors='none',edgecolors=c,alpha=.8,label=lab,lw=.7)
ax.set_xlim(0,240);ax.set_ylim(0,2600);ax.set_xlabel('Density k (veh/mi/lane)');ax.set_ylabel('Flow q (veh/h/lane)')
ax.annotate(f'IDM capacity ≈ {cap:,.0f} veh/h/lane\nat k ≈ {kc:.0f} veh/mi',xy=(kc,cap),xytext=(95,2250),arrowprops=dict(arrowstyle='->',lw=.7),fontsize=8.5)
ax.legend(loc='upper right',fontsize=8)
fig.tight_layout(pad=.4);fig.savefig('fig/f4.png');plt.close()

# ---- Fig 5 demand sweep
agg=json.load(open('agg1.json'))
qs=sorted(int(k) for k in agg)
fig,axs=plt.subplots(1,3,figsize=(6.5,2.6))
def band(ax,key,scale=1):
    xs=[q+200 for q in qs];mu=[np.mean(agg[str(q)][key])*scale for q in qs];lo=[np.min(agg[str(q)][key])*scale for q in qs];hi=[np.max(agg[str(q)][key])*scale for q in qs]
    ax.fill_between(xs,lo,hi,color='#3b8eea',alpha=.2,lw=0);ax.plot(xs,mu,'o-',color='#1a5fb4',ms=3.5,lw=1.2);return xs
band(axs[0],'cong',100);axs[0].set_ylabel('Congested intervals at A (%)')
band(axs[1],'nbd');axs[1].set_ylabel('Breakdown events per hour')
xs=band(axs[2],'dq');axs[2].plot([1200,4000],[1200,4000],'k:',lw=1);axs[2].set_ylabel('Mean flow at D (veh/h)')
axs[2].text(1300,3700,'1:1 line',fontsize=8)
for a in axs:a.set_xlabel('Offered load (veh/h)');a.set_xlim(1200,4000)
fig.tight_layout(pad=.4,w_pad=.8);fig.savefig('fig/f5.png');plt.close()

# ---- Fig 6 ramp sweep
def loadj(f):
    d=collections.defaultdict(list)
    for l in open(f):
        r=json.loads(l);d[r['over']]= d.get(r['over'],[]) if False else d[r['over'].get('ramp') if 'ramp' in r['over'] and 'meter' not in r['over'] else r['over'].get('meter')]
        d[r['over'].get('ramp') if 'meter' not in r['over'] else r['over']['meter']].append(r)
    return d
d2=collections.defaultdict(list)
for l in open('sw2.jsonl'):
    r=json.loads(l);d2[r['over']['ramp']].append(r)
for l in open('sw1.jsonl'):
    r=json.loads(l)
    if r['over']['flow']==2000:d2[200].append(r)
rs=sorted(d2)
mu=lambda k,s=1:[np.mean([x[k] for x in d2[q] if x[k] is not None])*s for q in rs]
fig,axs=plt.subplots(1,3,figsize=(6.5,2.6))
axs[0].plot(rs,[v for v in mu('merged')],'o-',color='#1a5fb4',ms=3.5);axs[0].plot([0,1200],[0,1200],'k:',lw=1)
axs[0].set_ylabel('Ramp throughput (veh/h)');axs[0].set_ylim(0,1250)
axs[1].plot(rs,mu('rampWait'),'o-',color='#c64600',ms=3.5);axs[1].set_ylabel('Mean ramp wait (s)')
axs[2].plot(rs,mu('blockedFrac',100),'o-',color='#a51d2d',ms=3.5);axs[2].set_ylabel('Ramp arrivals turned away (%)')
for a in axs:a.set_xlabel('Ramp demand (veh/h)')
fig.tight_layout(pad=.4,w_pad=.8);fig.savefig('fig/f6.png');plt.close()
json.dump({str(q):{k:float(np.mean([x[k] for x in d2[q] if x[k] is not None])) for k in ('merged','rampWait','blockedFrac','congFrac','nBD','dq','trip')} for q in rs},open('agg2.json','w'))

# ---- Fig 7 meter sweep
d3=collections.defaultdict(list)
for l in open('sw3.jsonl'):
    r=json.loads(l);d3[r['over']['meter']].append(r)
ms=sorted(d3)
m3=lambda k,s=1:[np.mean([x[k] for x in d3[q] if x[k] is not None])*s for q in ms]
fig,axs=plt.subplots(1,3,figsize=(6.5,2.6))
axs[0].plot(ms,m3('congFrac',100),'o-',color='#1a5fb4',ms=3.5);axs[0].set_ylabel('Congested intervals at A (%)')
axs[1].plot(ms,m3('blockedFrac',100),'o-',color='#a51d2d',ms=3.5);axs[1].set_ylabel('Ramp arrivals turned away (%)')
axs[2].plot(ms,m3('rampWait'),'o-',color='#c64600',ms=3.5);axs[2].set_ylabel('Mean ramp wait (s)')
for a in axs:a.set_xlabel('Meter wait (s/car)');a.set_xticks(ms)
fig.tight_layout(pad=.4,w_pad=.8);fig.savefig('fig/f7.png');plt.close()
json.dump({str(q):{k:float(np.mean([x[k] for x in d3[q] if x[k] is not None])) for k in ('merged','rampWait','blockedFrac','congFrac','nBD','dq','trip')} for q in ms},open('agg3.json','w'))

# ---- Fig 8 histogram
fig,ax=plt.subplots(figsize=(6.0,3.0))
gen=np.array(A['gen']);bins=np.arange(0,58,2)
ax.hist(gen,bins=bins,weights=np.ones(len(gen))/len(gen),color='#0f6b45',alpha=.75,label=f'Desired speeds generated (n = {len(gen):,})')
x=np.linspace(0,56,300);pdf=np.exp(-.5*((x-30)/5)**2)/(5*np.sqrt(2*np.pi))*2
ax.plot(x,pdf,color='#e8b92a',lw=2,label='Gaussian N(30, 5$^2$) mph')
act=np.array(A['act']);ax.plot(np.arange(len(act))*2+1,act/act.sum(),'o-',color='#3b8eea',ms=3,lw=1.5,label='Actual speeds (1 Hz samples)')
ax.set_xlabel('Speed (mph)');ax.set_ylabel('Fraction');ax.set_xlim(0,56);ax.legend()
fig.tight_layout(pad=.4);fig.savefig('fig/f8.png');plt.close()
print('ok',cap,kc)
