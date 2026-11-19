
function mk(){return new Proxy(function(){},{get:(t,p)=>p==='clientWidth'?1000:(p===Symbol.toPrimitive?()=>0:mk()),set:()=>true,apply:()=>mk()})}
global.document={getElementById:()=>mk(),documentElement:{},createElement:()=>mk(),body:mk()};
global.window={devicePixelRatio:1,addEventListener(){}};
global.getComputedStyle=()=>mk();
global.requestAnimationFrame=()=>{};
let _s=1;function seed(s){_s=s>>>0}
Math.random=function(){_s|=0;_s=_s+0x6D2B79F5|0;let t=Math.imul(_s^_s>>>15,1|_s);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296};
global.seed=seed;
(function(){
const M2F=3.28084,FT=4000*M2F,MPH=1.4667,CL=15,ROWS=4,ROWFT=FT/ROWS,DT=0.1,DINT=30; // road: x = 0-4,000 m (code works in ft internally)
const XA=1500*M2F,XD=3300*M2F; // detectors A and D at x = 1,500 m and 3,300 m
const IA=5,IB=9,S0=8;            // IDM: max accel, comfortable decel (ft/s²), standstill gap (ft)
const POL=0.2,ATHR=0.33;          // lane change: politeness, threshold (ft/s²)
const XB=100,NX=Math.ceil(FT/XB)+1,TB=2,NT=450; // x-t speed field: 100 ft × 2 s cells, 15 min kept
const $=id=>document.getElementById(id);
const P={flow:2000,mean:30,sd:5,headway:1,crash:1,clear:1,ppc:1.5,ramp:200,rampm:2500,rampL:500,rlen:400,brake:20,meter:2,nmax:20,bsafe:13,lc:true,speedup:1,running:true,xtl:0};
let lanes,alane,rampQ,queue,t,stepN,nextArr,nid,entered,exited,exits,tripSum,tripN,tripD,exitFt,extraFt,people,gen,act,actN,crash,nCrash,merged,waitSum,rampDemand,rampBlocked,lcCount,
    det,detLog,detLast,dHist,aHist,bd,bdEvents,wave,sfSum,sfCnt,lastCol,trajStore;

const avg=a=>a.length?a.reduce((x,y)=>x+y,0)/a.length:null;
const median=a=>{const s=a.slice().sort((x,y)=>x-y),m=s.length>>1;return s.length%2?s[m]:(s[m-1]+s[m])/2};
function gauss(){let u=0,v=0;while(!u)u=Math.random();v=Math.random();return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v)}
function expo(rate){return -Math.log(1-Math.random())/rate}
function geom(){const M=P.rampm*M2F,E=Math.min(M+P.rlen,FT-20),R0=Math.max(10,M-P.rampL*M2F);return{M,E,R0,xA:XA,xD:XD}} // R0 = ramp entry / meter stop line
const da=()=>({n:0,sv:0,si:0,occ:0});
function newDet(){det={A:[da(),da()],D:[da(),da()],B:{n:0,blk:0},C:{n:0,q:0,qmax:0,w:0}}}
function reset(){
  lanes=[[],[]];alane=[];rampQ=[];queue=0;t=0;stepN=0;nextArr=expo(P.flow/3600);nid=0;entered=0;exited=0;exits=[];tripSum=0;tripN=0;tripD=0;exitFt=0;extraFt=0;people=0;
  gen=[];act=new Array(28).fill(0);actN=0;crash=null;nCrash=0;merged=0;waitSum=0;rampDemand=0;rampBlocked=0;lcCount=0;
  detLog=[];detLast=[];dHist=[];aHist=[];bd={ev:null,below:0,above:0,free:0};bdEvents=[];wave=[null,null];
  sfSum=[new Float32Array(NT*NX),new Float32Array(NT*NX)];sfCnt=[new Uint16Array(NT*NX),new Uint16Array(NT*NX)];lastCol=-1;trajStore=[];
  newDet();uiDirty=true;
}

// IDM acceleration (also used for merge and lane-change safety checks)
function idm(v,v0,s,dv){return IA*(1-Math.pow(v/Math.max(v0,1),4)-Math.pow((S0+Math.max(0,v*P.headway+v*dv/(2*Math.sqrt(IA*IB))))/Math.max(0.5,s),2))}

// On-ramp: Poisson arrivals into storage (max n_max) -> meter -> acceleration lane -> zipper merge into the right lane
function ramp(){
  const {M,E,R0}=geom(),R=lanes[0];
  if(Math.random()<P.ramp/3600*DT){rampDemand++;
    if(rampQ.length<P.nmax){rampQ.push({t:t,v0:Math.max(8,P.mean+P.sd*gauss())*MPH});det.B.n++}else{rampBlocked++;det.B.blk++}}
  det.C.q+=rampQ.length*DT;det.C.qmax=Math.max(det.C.qmax,rampQ.length);
  if(rampQ.length){
    const head=rampQ[0],last=alane[alane.length-1];
    if(head.hs===undefined)head.hs=t;
    if(t-head.hs>=P.meter&&(!last||last.x-CL-R0>=S0)){ // meter releases one car from a stop at the ramp entry
      alane.push({id:nid++,x:R0,v:0,v0:head.v0,t0:t,ta:head.t,xs:R0,occ:P.ppc});rampQ.shift();det.C.n++;det.C.w+=t-head.t;
    }
  }
  const acc=alane.map((c,i)=>{
    let s=Math.max(0.5,E-c.x),dv=c.v;const l=alane[i-1];
    if(l){const s2=l.x-CL-c.x;if(s2<s){s=Math.max(0.5,s2);dv=c.v-l.v}}
    return Math.max(-P.brake,idm(c.v,c.v0,s,dv));
  });
  alane.forEach((c,i)=>{c.v=Math.max(0,c.v+acc[i]*DT);c.x+=c.v*DT;if(c.x>E){c.x=E;c.v=0}});
  // Zipper merge: only the front ramp car merges; never onto an occupied spot; holds to the lane end when traffic is slow;
  // each right-lane car lets in at most one ramp car.
  if(alane.length){
    const c=alane[0],n=R.length;let li=-1;
    if(c.x<M)return; // still on the ramp, not yet beside the main road
    for(let j=0;j<n;j++){if(R[j].x-CL>=c.x)li=j;else break}
    const lead=li>=0?R[li]:null,fol=li+1<n?R[li+1]:null;
    if(fol&&fol.x>c.x-CL)return;
    const forced=E-c.x<=150,bs=forced?P.brake:Math.min(P.brake,10+Math.min(10,0.25*(t-c.ta)));
    const lg=lead?lead.x-CL-c.x:1e9,fg=fol?c.x-CL-fol.x:1e9;
    const slow=(lead&&lead.v<25*MPH)||(fol&&fol.v<25*MPH);
    if(slow&&!forced)return;
    if(lg<(forced?3:8)||fg<(forced?3:8))return;
    if(lead&&idm(c.v,c.v0,lg,c.v-lead.v)<-bs)return;
    if(fol&&!fol.crashed&&idm(fol.v,fol.v0,fg,fol.v-c.v)<-(fol.gaveWay?1:bs))return;
    if(fol)fol.gaveWay=true;
    Object.assign(c,{gaveWay:true,lane:0,lat:-1,lcT:t,a:0,tr:[[t,c.x]]});
    R.splice(li+1,0,c);alane.shift();merged++;waitSum+=t-c.ta;
  }
}

function startCrash(){
  const all=lanes[0].concat(lanes[1]);
  if(crash||!all.length)return;
  const c=all[Math.floor(Math.random()*all.length)];
  c.crashed=true;c.v=0;crash={car:c,clearAt:t+P.clear*3600};nCrash++;
}

function mainAcc(){
  const {E}=geom(),fl=alane[0];
  for(let L=0;L<2;L++){const arr=lanes[L];
    for(let i=0;i<arr.length;i++){const c=arr[i];
      if(c.crashed){c.a=0;continue}
      const l=arr[i-1];let s=1e9,dv=0;
      if(l){s=l.x-CL-c.x;dv=c.v-l.v}
      if(L===0&&fl&&!c.gaveWay&&fl.x>E-250&&fl.x>c.x){const sf=fl.x-CL-c.x;if(sf<s){s=sf;dv=c.v-fl.v}}
      c.a=Math.max(-P.brake,idm(c.v,c.v0,Math.max(0.5,s),dv));
    }
  }
}

// MOBIL-style lane changes: go if own gain + politeness × followers' change > threshold, and the new follower brakes no harder than b_safe
function laneChanges(){
  if(!P.lc)return;
  const {E}=geom(),fl=alane[0];
  for(let L=0;L<2;L++){const T=1-L,tg=lanes[T];
    const snap=lanes[L].slice();let gone=0;
    for(let si=0;si<snap.length;si++){const c=snap[si];
      if(c.crashed||(c.id+stepN)%10||t-(c.lcT??-99)<4||c.x<CL+60)continue;
      const arr=lanes[L],i=si-gone,ol=arr[i-1],of=arr[i+1];
      let lo=0,hi=tg.length;while(lo<hi){const m=(lo+hi)>>1;if(tg[m].x>c.x)lo=m+1;else hi=m}const li=lo-1;
      const nl=li>=0?tg[li]:null,nf=tg[li+1]||null;
      let sN=1e9,dvN=0;
      if(nl){sN=nl.x-CL-c.x;if(sN<3)continue;dvN=c.v-nl.v}
      if(T===0&&fl&&fl.x>E-250&&fl.x>c.x){const sf=fl.x-CL-c.x;if(sf<sN){sN=Math.max(0.5,sf);dvN=c.v-fl.v}}
      const aNew=Math.max(-P.brake,idm(c.v,c.v0,sN,dvN));
      let dNf=0;
      if(nf){const g=c.x-CL-nf.x;if(g<3)continue;
        if(!nf.crashed){const an=idm(nf.v,nf.v0,g,nf.v-c.v);if(an<-P.bsafe)continue;dNf=an-nf.a}}
      let dOf=0;
      if(of&&!of.crashed){const an=ol?idm(of.v,of.v0,ol.x-CL-of.x,of.v-ol.v):idm(of.v,of.v0,1e9,0);dOf=an-of.a}
      if(aNew-c.a+POL*(dNf+dOf)<=ATHR)continue;
      arr.splice(i,1);tg.splice(li+1,0,c);
      if(c.tr&&c.tr.length)trajStore.push({L,p:c.tr});c.tr=[[t,c.x]];
      c.lane=T;c.lcT=t;c.lat=L-T;lcCount++;gone++;
    }
  }
}

function step(){
  stepN++;t=stepN*DT;
  const g=geom();
  if(stepN%10===0){for(const arr of lanes)for(const c of arr){if(c.crashed)continue;actN++;act[Math.min(27,Math.floor(c.v/MPH/2))]++}}
  while(t>=nextArr){queue++;nextArr+=expo(P.flow/3600)}
  // spawn at the entrance into whichever lane has more room
  for(let k=0;k<2&&queue>0;k++){
    const v0=Math.max(8,P.mean+P.sd*gauss())*MPH,ord=Math.random()<.5?[0,1]:[1,0];let best=-1,room=-1;
    for(const L of ord){const last=lanes[L][lanes[L].length-1],vs=last?Math.min(v0,last.v):v0,need=CL+8+vs*P.headway,r=last?last.x-CL:1e9;if(r>=need&&r>room){room=r;best=L}}
    if(best<0)break;
    const arr=lanes[best],last=arr[arr.length-1],vs=last?Math.min(v0,last.v):v0;
    arr.push({id:nid++,x:CL,v:vs,v0,t0:t,xs:CL,occ:P.ppc,lane:best,lat:0,a:0,tr:[[t,CL]]});queue--;entered++;gen.push(v0/MPH);
  }
  // crash: starts at random from the crash rate, blocks its lane until cleared
  if(crash){
    if(t>=crash.clearAt){const c=crash.car,arr=lanes[c.lane];extraFt+=Math.max(0,c.x-c.xs);arr.splice(arr.indexOf(c),1);if(c.tr)trajStore.push({L:c.lane,p:c.tr});crash=null}
  }else if(P.crash>0){
    let sv=0;for(const arr of lanes)for(const c of arr)sv+=c.v;
    if(sv>0&&Math.random()<P.crash*(sv*DT/5280)/1e6)startCrash();
  }
  ramp();
  mainAcc();
  // move cars and read point detectors A and D
  for(let L=0;L<2;L++){const dA=det.A[L],dD=det.D[L];let oA=false,oD=false;
    for(const c of lanes[L]){const px=c.x;
      if(c.crashed)c.v=0;else{c.v=Math.max(0,c.v+c.a*DT);c.x+=c.v*DT}
      if(px<g.xA&&c.x>=g.xA){dA.n++;dA.sv+=c.v;dA.si+=1/Math.max(c.v,0.1)}
      if(px<g.xD&&c.x>=g.xD){dD.n++;dD.sv+=c.v;dD.si+=1/Math.max(c.v,0.1)}
      if(c.x>=g.xA&&c.x-CL<=g.xA)oA=true;
      if(c.x>=g.xD&&c.x-CL<=g.xD)oD=true;
    }
    if(oA)dA.occ+=DT;if(oD)dD.occ+=DT;
  }
  laneChanges();
  for(const arr of lanes)for(const c of arr)if(c.lat)c.lat=c.lat>0?Math.max(0,c.lat-DT/8):Math.min(0,c.lat+DT/8); // slide lasts at most 8 sim-s (about 0.5 s real at 1×)
  for(let L=0;L<2;L++){const arr=lanes[L];
    while(arr.length&&arr[0].x-CL>FT){
      const c=arr.shift();exited++;people+=c.occ;const dist=FT+CL-c.xs;exitFt+=dist;tripD+=dist;exits.push(t);tripSum+=t-c.t0;tripN++;
      c.tr.push([t,c.x]);trajStore.push({L,p:c.tr});
    }
  }
  while(exits.length&&exits[0]<t-300)exits.shift();
  // x-t speed field and trajectories
  const col=Math.floor(t/TB+1e-9);
  if(col!==lastCol){for(let a=Math.max(lastCol+1,col-NT+1);a<=col;a++){const r=(a%NT)*NX;for(let L=0;L<2;L++){sfSum[L].fill(0,r,r+NX);sfCnt[L].fill(0,r,r+NX)}}lastCol=col}
  if(stepN%10===0){const r=(col%NT)*NX;for(let L=0;L<2;L++)for(const c of lanes[L]){const xb=Math.min(NX-1,Math.max(0,Math.floor(c.x/XB)));sfSum[L][r+xb]+=c.v/MPH;sfCnt[L][r+xb]++}}
  if(stepN%20===0){for(const arr of lanes)for(const c of arr)c.tr.push([t,c.x]);
    if(stepN%600===0)trajStore=trajStore.filter(s=>s.p[s.p.length-1][0]>t-NT*TB)}
  if(stepN%300===0)closeInterval();
}

// ---------- detectors, breakdown, wave speed ----------
function closeInterval(){
  const t1=Math.round(t),t0=t1-DINT,g=geom(),rows=[];
  const mk=(id,L,d,x)=>{const q=d.n*3600/DINT,ut=d.n?d.sv/d.n/MPH:null,us=d.n?d.n/d.si/MPH:null,occ=d.occ/DINT;let k,flag='OK';
    if(d.n>=2&&us>0.5)k=q/us;else{k=occ*5280/CL;flag=d.n?'K_FROM_OCC':(occ>0.05?'STOPPED':'NO_VEH')}
    return{sensor:id,lane:L?'L':'R',x_ft:Math.round(x),t0,t1,count:d.n,flow_vph:q,tmean_mph:ut,smean_mph:us,density_vpm:k,occ_pct:occ*100,flag}};
  rows.push(mk('A',0,det.A[0],g.xA),mk('A',1,det.A[1],g.xA),mk('D',0,det.D[0],g.xD),mk('D',1,det.D[1],g.xD));
  rows.push({sensor:'B',lane:'ramp',x_ft:Math.round(g.M),t0,t1,count:det.B.n,flow_vph:det.B.n*3600/DINT,blocked:det.B.blk,flag:det.B.blk?'SPILLBACK':'OK'});
  rows.push({sensor:'C',lane:'ramp',x_ft:Math.round(g.M),t0,t1,count:det.C.n,flow_vph:det.C.n*3600/DINT,avg_queue:det.C.q/DINT,max_queue:det.C.qmax,avg_wait_s:det.C.n?det.C.w/det.C.n:null,flag:det.C.qmax>=P.nmax?'FULL':'OK'});
  newDet();detLog.push(...rows);detLast=rows;uiDirty=true;
  // breakdown at A (right lane), flows measured at D (both lanes)
  const aR=rows[0],dq=rows[2].flow_vph+rows[3].flow_vph;
  const uA=aR.smean_mph!=null?aR.smean_mph:(aR.flag==='STOPPED'?0:P.mean);
  dHist.push(dq);aHist.push({q:aR.flow_vph,k:aR.density_vpm,u:uA});
  const thr=0.6*P.mean,n=dHist.length,warm=t>=600; // skip the first 10 min while the 4 km road fills
  if(!bd.ev){
    if(uA<thr&&warm){bd.below++;
      if(bd.below>=3){const s=n-3,fr=bd.free;const pre=fr>=4?dHist.slice(Math.max(0,s-Math.min(10,fr)),s):[]; // only free-flow intervals count as pre-breakdown
        const ev={s,t:t0-DINT,endIdx:null,end:null,pre:pre.length?avg(pre):null,crash:!!crash};bdEvents.push(ev);bd={ev,below:0,above:0,free:0}}}
    else{bd.below=0;if(warm)bd.free++}
  }else{
    if(crash)bd.ev.crash=true;
    if(uA>=thr){bd.above++;if(bd.above>=2){bd.ev.endIdx=n-2;bd.ev.end=t0-DINT;updEv(bd.ev);bd={ev:null,below:0,above:0,free:2}}}
    else bd.above=0;
  }
  if(bd.ev)updEv(bd.ev);
  if(stepN%600===0)wave=[waveSpeed(0),waveSpeed(1)];
}
function updEv(ev){
  const e=ev.endIdx!=null?ev.endIdx:dHist.length;
  const dq=dHist.slice(ev.s+2,e);ev.qdf=dq.length?avg(dq):null;
  ev.drop=(ev.pre&&ev.qdf!=null)?100*(1-ev.qdf/ev.pre):null;
  const fr=aHist.slice(Math.max(0,ev.s-4),ev.s),cg=aHist.slice(ev.s,Math.min(e,ev.s+4));
  ev.w=null;
  if(ev.pre!=null&&fr.length&&cg.length){const qf=avg(fr.map(a=>a.q)),kf=avg(fr.map(a=>a.k)),qc=avg(cg.map(a=>a.q)),kc=avg(cg.map(a=>a.k));if(kc-kf>10){const w=(qf-qc)/(kf-kc);if(Math.abs(w)<60)ev.w=w}}
}
// cross-correlate speed series 800 ft apart; the lag that best lines them up gives the wave speed
function waveSpeed(L){
  const W=Math.min(300,lastCol-1);if(W<90)return null;
  const a0=lastCol-W,SR=[];
  for(let xb=0;xb<NX;xb++){const s=new Float32Array(W);for(let k=0;k<W;k++){const r=((a0+k)%NT)*NX+xb,c=sfCnt[L][r];s[k]=c?sfSum[L][r]/c:NaN}SR.push(s)}
  const D=8,ws=[];
  for(let i=0;i+D<NX;i+=2){const A=SR[i],B=SR[i+D];
    let m=0,c=0;for(const v of A)if(v===v){m+=v;c++}if(c<W*0.6)continue;m/=c;
    let sd=0;for(const v of A)if(v===v)sd+=(v-m)*(v-m);sd=Math.sqrt(sd/c);
    if(m>0.6*P.mean||sd<2)continue; // congested cells only
    let best=0,bl=0;
    for(let lag=1;lag<=50;lag++){let n=0,sa=0,sb=0,saa=0,sbb=0,sab=0;
      for(let k=Math.max(0,lag);k<W&&k<W+lag;k++){const x=A[k],y=B[k-lag];if(x!==x||y!==y)continue;n++;sa+=x;sb+=y;saa+=x*x;sbb+=y*y;sab+=x*y}
      if(n<40)continue;const ma=sa/n,mb=sb/n,va=saa/n-ma*ma,vb=sbb/n-mb*mb;if(va<=0||vb<=0)continue;
      const r=(sab/n-ma*mb)/Math.sqrt(va*vb);if(r>best){best=r;bl=lag}}
    if(best>0.6&&bl!==0)ws.push(-(D*XB)/(bl*TB)/MPH);
  }
  return ws.length>=2?{w:median(ws),n:ws.length}:null;
}
function idmEq(){const v0=P.mean*MPH,pts=[];for(let i=0;i<=240;i++){const v=v0*i/240*0.9995,se=(S0+v*P.headway)/Math.sqrt(1-Math.pow(v/v0,4)),k=5280/(se+CL);pts.push([k,k*v/MPH])}return pts}

// ---------- drawing ----------
const road=$('road'),rc=road.getContext('2d'),xt=$('xt'),xc=xt.getContext('2d'),qk=$('qk'),qc=qk.getContext('2d'),hist=$('hist'),hc=hist.getContext('2d');
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function fit(cv,h){const d=window.devicePixelRatio||1,w=cv.parentElement.clientWidth-20;cv.width=w*d;cv.height=h*d;cv.style.height=h+'px';cv.getContext('2d').setTransform(d,0,0,d,0,0);return w}
let W=0,HW=0,XW=0,QW=0,uiDirty=true;
function resize(){W=fit(road,390);XW=fit(xt,330);QW=fit(qk,300);HW=fit(hist,200)}
const PAL=[];for(let i=0;i<=60;i++)PAL.push(`hsl(${5+Math.min(1,Math.max(0,(i-5)/45))*195},80%,52%)`);
const spc=mph=>PAL[Math.max(0,Math.min(60,Math.round(mph)))];
const LH=18,ROAD=LH*2,GAP=60,RH=ROAD+GAP,PAD=24;
const rowY=r=>r*RH+6;
const laneY=(r,p)=>p>=0?rowY(r)+27-18*p:rowY(r)+27-20*p; // p: 1 left, 0 right, -1 accel lane
const ease=u=>u*u*(3-2*u);
function drawRoad(){
  const ctx=rc,len=W-PAD*2,k=len/ROWFT,g=geom();
  ctx.clearRect(0,0,W,400);ctx.font='600 12px Barlow,sans-serif';
  for(let r=0;r<ROWS;r++){
    const y=rowY(r);ctx.fillStyle=css('--road');ctx.fillRect(PAD,y,len,ROAD);
    ctx.strokeStyle='rgba(255,255,255,.55)';ctx.lineWidth=1.5;ctx.setLineDash([10,10]);ctx.beginPath();ctx.moveTo(PAD,y+LH);ctx.lineTo(PAD+len,y+LH);ctx.stroke();ctx.setLineDash([]);
    ctx.fillStyle='#c9cdca';ctx.fillText((r*1000).toLocaleString()+' m',PAD+4,y+13);
  }
  ctx.fillStyle=css('--muted');ctx.textAlign='right';ctx.fillText('C: exit at 4,000 m',PAD+len,ROWS*RH-6);ctx.textAlign='left';
  // detectors A and D
  for(const [nm,xd] of [['A',g.xA],['D',g.xD]]){const r=Math.min(ROWS-1,Math.floor(xd/ROWFT)),px=PAD+(xd-r*ROWFT)*k,y=rowY(r);
    ctx.fillStyle=css('--mark');ctx.fillRect(px-1.5,y-3,3,ROAD+6);ctx.font='700 12px Barlow,sans-serif';ctx.fillText(nm,px+4,y+ROAD+13);ctx.font='600 12px Barlow,sans-serif'}
  // ramp: entry/meter at R0, ramp road R0->M drawn just below the main road, acceleration lane M->E attached to it
  const M=g.M,E=g.E,R0=g.R0,rowOf=x=>Math.min(ROWS-1,Math.floor(x/ROWFT)),pxOf=x=>PAD+(x-rowOf(x)*ROWFT)*k;
  const seg=(a0,a1,fn)=>{for(let r2=0;r2<ROWS;r2++){const a=Math.max(a0,r2*ROWFT),e=Math.min(a1,(r2+1)*ROWFT);if(e>a)fn(r2,PAD+(a-r2*ROWFT)*k,(e-a)*k,e===a1)}};
  seg(R0,M,(r2,xa,w)=>{const yy=rowY(r2)+ROAD+6;ctx.fillStyle=css('--sign');ctx.fillRect(xa,yy-2,w,22);ctx.fillStyle=css('--road');ctx.fillRect(xa,yy,w,18)});
  seg(M,E,(r2,xa,w,last)=>{const yy=rowY(r2)+ROAD;ctx.fillStyle=css('--sign');ctx.fillRect(xa-1,yy,w+2,25);ctx.fillStyle=css('--road');ctx.fillRect(xa,yy,w,22);if(last){ctx.fillStyle=css('--mark');ctx.fillRect(xa+w-3,yy,3,22)}});
  const rr=rowOf(R0),bx=pxOf(R0),qy=rowY(rr)+ROAD+38;
  const shown=Math.min(rampQ.length,Math.max(0,Math.floor((W-bx-10)/20)));
  for(let i=0;i<shown;i++){ctx.fillStyle=spc(rampQ[i].v0/MPH);ctx.fillRect(bx+10+i*20,qy-6,17,12)}
  for(const c of alane){ctx.fillStyle=spc(c.v/MPH);
    seg(Math.max(R0,c.x-CL),c.x,(r2,xa,w)=>{const yy=rowY(r2)+ROAD+(c.x<M?15:11);ctx.fillRect(xa,yy-6,w-1,12)})}
  ctx.fillStyle=(rampQ.length&&rampQ[0].hs!==undefined&&t-rampQ[0].hs>=P.meter)?'#2fbf71':'#e5252a';ctx.beginPath();ctx.arc(bx+3,qy,4,0,6.283);ctx.fill();
  ctx.fillStyle=css('--mark');ctx.font='700 12px Barlow,sans-serif';ctx.fillText('B, C',Math.max(2,bx-28),qy+4);
  ctx.fillStyle=css('--muted');ctx.font='600 12px Barlow,sans-serif';
  ctx.fillText('Ramp storage '+rampQ.length+'/'+P.nmax+' · meter '+P.meter.toFixed(1)+' s · ramp '+P.rampL+' m',Math.min(bx+10+Math.max(shown,1)*20+8,W-300),qy+4);
  // mainline cars
  const cw=CL*k;
  for(let L=0;L<2;L++)for(const c of lanes[L]){
    ctx.fillStyle=spc(c.v/MPH);
    const p=L+(c.lat?Math.sign(c.lat)*ease(Math.abs(c.lat)):0);
    for(let r2=0;r2<ROWS;r2++){const a=Math.max(c.x-CL,r2*ROWFT),e=Math.min(c.x,(r2+1)*ROWFT);
      if(e>a){const y=laneY(r2,p);ctx.fillRect(PAD+(a-r2*ROWFT)*k,y-6,(e-a)*k-(cw>6?1:0),12)}}
  }
  if(crash){const c=crash.car,r2=Math.min(ROWS-1,Math.floor((c.x-1)/ROWFT)),y=laneY(r2,c.lane),px=PAD+(c.x-r2*ROWFT)*k;
    ctx.fillStyle='#fff';ctx.strokeStyle='#e5252a';ctx.lineWidth=2;ctx.fillRect(px-cw-1,y-6,cw+2,12);ctx.strokeRect(px-cw-1,y-6,cw+2,12);
    ctx.fillStyle='#e5252a';ctx.font='700 12px Barlow,sans-serif';ctx.textAlign='center';ctx.fillText('CRASH',px-cw/2,c.lane?y-9:y+19);ctx.textAlign='left'}
}
function axes(ctx,L,Tp,w,h,xt,yt){ctx.strokeStyle=css('--line');ctx.lineWidth=1;ctx.strokeRect(L,Tp,w,h);ctx.fillStyle=css('--muted');ctx.font='12px Barlow,sans-serif';
  ctx.textAlign='center';ctx.fillText(xt,L+w/2,Tp+h+28);ctx.save();ctx.translate(12,Tp+h/2);ctx.rotate(-Math.PI/2);ctx.fillText(yt,0,0);ctx.restore();ctx.textAlign='left'}
function drawXT(){
  const ctx=xc,H=330,L=50,B=36,Tp=8,w=XW-L-10,h=H-B-Tp,Ln=P.xtl,span=NT*TB,t0=Math.max(0,t-span),g=geom();
  ctx.clearRect(0,0,XW,H);
  const X=tt=>L+(tt-t0)/span*w,Y=x=>Tp+h-x/FT*h;
  ctx.save();ctx.beginPath();ctx.rect(L,Tp,w,h);ctx.clip();
  const cw=w/NT+0.7,ch=h*XB/FT+0.7;
  for(let a=Math.max(0,Math.ceil(t0/TB));a<=lastCol;a++){const r=(a%NT)*NX,px=X(a*TB);
    for(let xb=0;xb<NX;xb++){const n=sfCnt[Ln][r+xb];if(!n)continue;ctx.fillStyle=spc(sfSum[Ln][r+xb]/n);ctx.fillRect(px,Y((xb+1)*XB),cw,ch)}}
  ctx.strokeStyle=css('--traj');ctx.lineWidth=0.8;ctx.beginPath();
  const path=(p,cur)=>{let st=false;for(const [tt,x] of p){if(tt<t0)continue;st?ctx.lineTo(X(tt),Y(x)):ctx.moveTo(X(tt),Y(x));st=true}if(cur&&st)ctx.lineTo(X(t),Y(cur.x))};
  for(const s of trajStore)if(s.L===Ln)path(s.p);
  for(const c of lanes[Ln])path(c.tr,c);
  ctx.stroke();
  ctx.setLineDash([5,5]);ctx.lineWidth=1;ctx.font='600 11px Barlow,sans-serif';
  const ref=[['A',g.xA],['D',g.xD]];if(Ln===0)ref.push(['merge',g.M],['accel. lane end',g.E]);ref.sort((a,b)=>a[1]-b[1]);
  for(const [nm,x] of ref){ctx.strokeStyle=css('--ink');ctx.globalAlpha=.6;ctx.beginPath();ctx.moveTo(L,Y(x));ctx.lineTo(L+w,Y(x));ctx.stroke();ctx.globalAlpha=1;ctx.fillStyle=css('--ink');ctx.fillText(nm,L+4,Y(x)-3)}
  ctx.setLineDash([]);ctx.restore();
  axes(ctx,L,Tp,w,h,'Time (min)','Position x (m)');
  ctx.fillStyle=css('--muted');ctx.font='12px Barlow,sans-serif';ctx.textAlign='right';
  for(let m=0;m<=4000;m+=1000)ctx.fillText(m.toLocaleString(),L-4,Y(m*M2F)+4);
  ctx.textAlign='center';for(let s=Math.ceil(t0/120)*120;s<=t0+span;s+=120)ctx.fillText((s/60).toFixed(0),X(s),Tp+h+14);ctx.textAlign='left';
}
function drawQK(){
  const ctx=qc,H=300,L=56,B=36,Tp=10,w=QW-L-12,h=H-B-Tp;ctx.clearRect(0,0,QW,H);
  const eq=idmEq(),cap=Math.max(...eq.map(p=>p[1])),kmax=240,qmax=Math.max(2600,Math.ceil(cap/500)*500+500);
  const X=k=>L+Math.min(k,kmax)/kmax*w,Y=q=>Tp+h-q/qmax*h;
  ctx.fillStyle=css('--muted');ctx.font='12px Barlow,sans-serif';ctx.strokeStyle=css('--line');ctx.lineWidth=1;
  ctx.textAlign='right';for(let q=0;q<=qmax;q+=500){ctx.beginPath();ctx.moveTo(L,Y(q));ctx.lineTo(L+w,Y(q));ctx.stroke();ctx.fillText(q,L-4,Y(q)+4)}
  ctx.textAlign='center';for(let k=0;k<=kmax;k+=40)ctx.fillText(k,X(k),Tp+h+14);ctx.textAlign='left';
  for(const r of detLog){if(r.sensor!=='A'&&r.sensor!=='D')continue;const col=r.sensor==='A'?'#e07b39':'#3b8eea';
    ctx.beginPath();ctx.arc(X(r.density_vpm),Y(r.flow_vph),3,0,6.283);
    if(r.lane==='R'){ctx.globalAlpha=.65;ctx.fillStyle=col;ctx.fill()}else{ctx.globalAlpha=.8;ctx.strokeStyle=col;ctx.lineWidth=1.5;ctx.stroke()}ctx.globalAlpha=1}
  ctx.strokeStyle=css('--mark');ctx.lineWidth=2.5;ctx.beginPath();eq.forEach((p,i)=>i?ctx.lineTo(X(p[0]),Y(p[1])):ctx.moveTo(X(p[0]),Y(p[1])));ctx.stroke();
  const pk=eq.find(p=>p[1]===cap);ctx.fillStyle=css('--ink');ctx.font='600 12px Barlow,sans-serif';
  ctx.fillText('IDM capacity ≈ '+Math.round(cap).toLocaleString()+' veh/h/lane at k ≈ '+Math.round(pk[0]),Math.min(X(pk[0])+6,L+w-260),Y(cap)-6);
  axes(ctx,L,Tp,w,h,'Density k (veh/mi/lane)','Flow q (veh/h/lane)');
}
function drawHist(){
  const ctx=hc,L=34,B=24,Tp=8,w=HW-L-8,h=200-B-Tp,lo=0,hi=56,bw=2,nb=(hi-lo)/bw;
  ctx.clearRect(0,0,HW,200);
  const bins=new Array(nb).fill(0);
  for(const v of gen){const i=Math.floor((v-lo)/bw);if(i>=0&&i<nb)bins[i]++}
  const n=Math.max(1,gen.length),pdf=x=>Math.exp(-0.5*Math.pow((x-P.mean)/P.sd,2))/(P.sd*Math.sqrt(2*Math.PI));
  const an=Math.max(1,actN),ymax=Math.max(pdf(P.mean)*bw,...bins.map(b=>b/n),...act.map(b=>b/an))*1.15;
  ctx.fillStyle=css('--sign');ctx.globalAlpha=.75;
  bins.forEach((b,i)=>{const bh=b/n/ymax*h;ctx.fillRect(L+i*w/nb+1,Tp+h-bh,w/nb-2,bh)});
  ctx.globalAlpha=1;ctx.strokeStyle=css('--mark');ctx.lineWidth=2;ctx.beginPath();
  for(let x=lo;x<=hi;x+=.5){const px=L+(x-lo)/(hi-lo)*w,py=Tp+h-pdf(x)*bw/ymax*h;x===lo?ctx.moveTo(px,py):ctx.lineTo(px,py)}
  ctx.stroke();
  if(actN){ctx.strokeStyle='#3b8eea';ctx.fillStyle='#3b8eea';ctx.lineWidth=2.5;ctx.beginPath();
    act.forEach((b,i)=>{const px=L+(i+.5)*w/nb,py=Tp+h-b/an/ymax*h;i?ctx.lineTo(px,py):ctx.moveTo(px,py)});ctx.stroke();
    act.forEach((b,i)=>{ctx.beginPath();ctx.arc(L+(i+.5)*w/nb,Tp+h-b/an/ymax*h,2.5,0,6.283);ctx.fill()})}
  ctx.fillStyle=css('--muted');ctx.font='12px Barlow,sans-serif';ctx.textAlign='center';
  for(let x=lo;x<=hi;x+=10)ctx.fillText(x+' mph',L+(x-lo)/(hi-lo)*w,200-6);
  ctx.textAlign='left';ctx.strokeStyle=css('--line');ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(L,Tp+h);ctx.lineTo(L+w,Tp+h);ctx.stroke();
}
const f=(v,d=0)=>v==null||!isFinite(v)?'–':(+v).toLocaleString(undefined,{minimumFractionDigits:d,maximumFractionDigits:d});
const clk=s=>{s=Math.round(s);return Math.floor(s/3600)+':'+String(Math.floor(s%3600/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0')};
function detUI(){
  if(!uiDirty)return;uiDirty=false;const g=geom();
  $('detNote').innerHTML=`A at x = 1,500 m (main road, upstream of the merge), B at the ramp entry (x = ${Math.round(g.R0/M2F).toLocaleString()} m), C on the ramp queue at the meter stop line, D at x = 3,300 m (main road, downstream). Point C, the destination, is the road exit at x = 4,000 m. Speed is the space-mean speed (harmonic mean of spot speeds). Density is q / u<sub>s</sub>, or comes from occupancy when fewer than two cars pass.`;
  let h='<tr><th>Sensor</th><th>Lane</th><th>Count</th><th>Flow (veh/h)</th><th>Speed (mph)</th><th>Density (veh/mi)</th><th>Occupancy / queue</th><th>Flag</th></tr>';
  for(const r of detLast){
    if(r.sensor==='A'||r.sensor==='D')h+=`<tr><td>${r.sensor}</td><td>${r.lane==='R'?'Right':'Left'}</td><td>${r.count}</td><td>${f(r.flow_vph)}</td><td>${f(r.smean_mph,1)}</td><td>${f(r.density_vpm)}</td><td>${f(r.occ_pct)}%</td><td>${r.flag}</td></tr>`;
    else if(r.sensor==='B')h+=`<tr><td>B</td><td>Ramp entry</td><td>${r.count}</td><td>${f(r.flow_vph)}</td><td>–</td><td>–</td><td>${r.blocked} turned away</td><td>${r.flag}</td></tr>`;
    else h+=`<tr><td>C</td><td>Ramp meter</td><td>${r.count}</td><td>${f(r.flow_vph)}</td><td>–</td><td>–</td><td>avg ${f(r.avg_queue,1)}, max ${r.max_queue} · wait ${f(r.avg_wait_s,0)} s</td><td>${r.flag}</td></tr>`;
  }
  $('detT').innerHTML=h;
  $('detTime').textContent=detLast.length?`Latest interval ${clk(detLast[0].t0)}–${clk(detLast[0].t1)} · ${detLog.length/6} intervals logged`:'Waiting for first 30 s interval…';
  const ev=bdEvents[bdEvents.length-1];
  $('bS').textContent=bd.ev?'Broken down':'Free flow';
  $('bF').textContent=ev&&ev.pre!=null?f(ev.pre)+' /hr':'–';
  $('bQ').textContent=ev&&ev.qdf!=null?f(ev.qdf)+' /hr':'–';
  $('bDr').textContent=ev&&ev.drop!=null?f(ev.drop,1)+'%':'–';
  $('wS').textContent=ev&&ev.w!=null?f(ev.w,1)+' mph':'–';
  $('wX').textContent=(wave[0]?f(wave[0].w,1):'–')+' / '+(wave[1]?f(wave[1].w,1):'–')+' mph';
  $('wN').textContent=f(-(CL+S0)/P.headway/MPH,1)+' mph';
  let b='<tr><th>#</th><th>Start</th><th>End</th><th>Breakdown flow (veh/h)</th><th>Queue discharge (veh/h)</th><th>Capacity drop</th><th>Shockwave ω (mph)</th><th>Crash active</th></tr>';
  if(!bdEvents.length)b+='<tr><td colspan="8" style="text-align:left;color:var(--muted)">No breakdowns yet. Try raising main road demand or ramp flow, or trigger a crash.</td></tr>';
  bdEvents.forEach((e,i)=>{b+=`<tr><td>${i+1}</td><td>${clk(e.t)}</td><td>${e.end!=null?clk(e.end):'ongoing'}</td><td>${e.pre!=null?f(e.pre):'congested from start'}</td><td>${f(e.qdf)}</td><td>${e.drop!=null?f(e.drop,1)+'%':'–'}</td><td>${f(e.w,1)}</td><td>${e.crash?'Yes':'No'}</td></tr>`});
  $('bdT').innerHTML=b;
}
function stats(){
  $('sClock').textContent=clk(Math.floor(t));
  const all=lanes[0].concat(lanes[1]);
  $('sOn').textContent=all.length;$('sQ').textContent=queue;$('sIO').textContent=entered+' / '+exited;
  $('sPP').textContent=Math.round(people).toLocaleString();
  const mv=a=>a.length?(a.reduce((s,c)=>s+c.v,0)/a.length/MPH).toFixed(0):'–';
  $('sV').textContent=all.length?mv(lanes[0])+' / '+mv(lanes[1])+' mph':'–';
  $('sD').textContent=(all.length/(FT/5280)/2).toFixed(0)+' cars/mi/lane';
  $('sF').textContent=t>60?Math.round(exits.length*3600/Math.min(300,t))+' /hr':'–';
  const useM=t>300,fl=useM?exits.length*12:P.flow+P.ramp;
  $('sAL').textContent=useM?'Projected AADT (measured × 24)':'Projected AADT (input × 24)';
  $('sA').textContent=Math.round(fl*24).toLocaleString();
  const vmt=(exitFt+extraFt+all.reduce((a,c)=>a+Math.max(0,c.x-c.xs),0)+alane.reduce((a,c)=>a+Math.max(0,c.x-c.xs),0))/5280;
  $('sVM').textContent=vmt.toFixed(1);
  const avgMi=exited?exitFt/exited/5280:FT/5280;
  $('sVD').textContent=Math.round(fl*24*avgMi).toLocaleString();
  $('sCr').textContent=nCrash;
  if(crash){const r=Math.max(0,crash.clearAt-t);$('sCS').textContent=(crash.car.lane?'Left':'Right')+' blocked, '+Math.floor(r/3600)+'h '+String(Math.floor(r%3600/60)).padStart(2,'0')+'m'}else $('sCS').textContent='Clear';
  $('sCY').textContent=(fl*24*365*avgMi*P.crash/1e6).toFixed(1);
  $('sLC').textContent=lcCount.toLocaleString();
  $('sRQ').textContent=rampQ.length+' / '+P.nmax;$('sRB').textContent=rampBlocked;$('sLN').textContent=alane.length;$('sRM').textContent=merged;
  $('sRW').textContent=merged?(waitSum/merged).toFixed(1)+' s':'–';
  $('sT').textContent=tripN?(tripSum/tripN).toFixed(0)+' s ('+(tripD/tripSum/MPH).toFixed(1)+' mph)':'–';
}
function csv(){
  const cols=['sensor','lane','x_ft','t0','t1','count','flow_vph','tmean_mph','smean_mph','density_vpm','occ_pct','blocked','avg_queue','max_queue','avg_wait_s','flag'];
  const head=['sensor_id','lane','x_ft','interval_start_s','interval_end_s','count','flow_vph','time_mean_speed_mph','space_mean_speed_mph','density_veh_per_mi','occupancy_pct','turned_away','avg_queue','max_queue','avg_wait_s','quality_flag'];
  const out=[head.join(',')].concat(detLog.map(r=>cols.map(c=>{const v=r[c];return v==null?'':(typeof v==='number'?+v.toFixed(3):v)}).join(',')));
  const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([out.join('\n')],{type:'text/csv'}));a.download='detectors.csv';document.body.appendChild(a);a.click();a.remove();
}

// ---------- loop ----------
let last=performance.now(),accT=0,lastHeavy=0;
function frame(now){
  const rdt=Math.min(0.1,(now-last)/1000);last=now;
  if(P.running){accT+=rdt*P.speedup*15;let n=0;while(accT>=DT&&n<1500){step();accT-=DT;n++}if(n>=1500)accT=0}
  const dec=rdt/0.5; // lane-change and ramp slides take 0.5 s of real time at any playback speed
  for(const arr of lanes)for(const c of arr)if(c.lat)c.lat=c.lat>0?Math.max(0,c.lat-dec):Math.min(0,c.lat+dec);
  drawRoad();stats();
  if(now-lastHeavy>300){lastHeavy=now;drawXT();drawQK();drawHist();detUI()}
  requestAnimationFrame(frame);
}

// ---------- controls ----------
$('play').onclick=()=>{P.running=!P.running;$('play').textContent=P.running?'Pause':'Play'};
$('reset').onclick=reset;
$('crashnow').onclick=startCrash;
$('csv').onclick=csv;
$('speedup').onchange=e=>P.speedup=+e.target.value;
$('xtl').onchange=e=>P.xtl=+e.target.value;
$('lc').onchange=e=>P.lc=e.target.checked;
const one=new Set(['headway','crash','clear','ppc','meter']);
[['flow','flowv'],['mean','meanv'],['sd','sdv'],['headway','hwv'],['crash','crv'],['clear','clv'],['ppc','ppv'],['ramp','rfv'],['rampm','rmv'],['rampL','rLv'],['rlen','rlv'],['brake','mbv'],['meter','mtv'],['nmax','nmv'],['bsafe','bsv']].forEach(([k,o])=>{
  $(k).oninput=e=>{const v=+e.target.value;P[k]=v;$(o).textContent=k==='rampm'?v.toLocaleString():one.has(k)?v.toFixed(1):e.target.value;
    if(k==='mean'||k==='sd'){gen=[];act.fill(0);actN=0}
    if(k==='rampm'||k==='rlen'||k==='rampL'){alane.slice().reverse().forEach(c=>rampQ.unshift({t:c.ta,v0:c.v0}));alane=[]}
    uiDirty=true}
});
window.addEventListener('resize',resize);
globalThis.__sim={P,reset,step,idmEq,waveSpeed,startCrash,geom,
 S:()=>({crash,t,stepN,detLog,bdEvents,lanes,alane,rampQ,rampBlocked,rampDemand,merged,waitSum,exited,entered,tripSum,tripN,tripD,lcCount,queue,nCrash,gen,act,actN,sfSum,sfCnt,lastCol,wave,trajStore,exitFt,people}),
 consts:{NX,NT,TB,XB,FT,M2F,MPH,CL,S0,IA,IB,POL,ATHR}};
})();
