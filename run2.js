require('./sim.js');
const {P,reset,step,S,startCrash,geom,consts}=__sim;
const {MPH,CL,M2F,FT}=consts;
const DEF=JSON.parse(JSON.stringify(P));
const fs=require('fs');
function run2(over,sd,opt={}){
  const dur=opt.dur||3600,WU=600;
  Object.assign(P,DEF,{crash:0},over);seed(sd);reset();
  const N=Math.round(dur/0.1),W0=WU*10;
  let snap=null,tet=0,vm=0,hb9=0,hb15=0,lastMerged=0,mergeRecs=[],crashed=false,crashInfo=null,wExit=[];
  for(let i=0;i<N;i++){
    step();
    const s=S();const t=s.t;
    if(opt.crashAt&&!crashed&&t>=opt.crashAt){startCrash();crashed=true;const cr=S().crash;if(cr)crashInfo={lane:cr.car.lane,x:cr.car.x/M2F,t}}
    if(i===W0-1){snap={ex:s.exited,pp:s.people,mg:s.merged,bl:s.rampBlocked,rd:s.rampDemand,ts:s.tripSum,tn:s.tripN,td:s.tripD,ws:s.waitSum,lc:s.lcCount,q:s.queue,en:s.entered}}
    if(i>=W0){
      for(let L=0;L<2;L++){const arr=s.lanes[L];
        for(let k=0;k<arr.length;k++){const c=arr[k];vm+=c.v*0.1/5280;
          if(!c.crashed){
            if(c.a<-9){if(!c._h){c._h=1;hb9++}}else if(c._h&&c.a>-6)c._h=0;
            if(c.a<-15){if(c._s!==1){c._s=1;hb15++}}else if(c._s&&c.a>-10)c._s=0;
          }
          if(k>0){const l=arr[k-1],gap=l.x-CL-c.x,dv=c.v-l.v;if(dv>0.5&&gap>0&&gap/dv<2)tet+=0.1}
        }}
      for(const c of s.alane)vm+=c.v*0.1/5280;
      if(s.merged>lastMerged){const R=s.lanes[0];const idx=R.findIndex(c=>c.lcT===t&&c.lat<0&&c.ta!==undefined);
        if(idx>=0){const c=R[idx],l=R[idx-1],f=R[idx+1],g=geom();
          const fg=f?c.x-CL-f.x:null,lg=l?l.x-CL-c.x:null,fdv=f?f.v-c.v:0;
          mergeRecs.push({fg,lg,fttc:(f&&fdv>0.5)?fg/fdv:null,forced:c.x>=g.E-150,v:c.v/MPH})}}
    }
    lastMerged=s.merged;
  }
  const s=S(),Wh=(dur-WU)/3600;
  const ex=s.exited-snap.ex,pp=s.people-snap.pp,mg=s.merged-snap.mg,tn=s.tripN-snap.tn;
  const tripMean=tn?(s.tripSum-snap.ts)/tn:null,tripV=(s.tripSum-snap.ts)?(s.tripD-snap.td)/(s.tripSum-snap.ts)/MPH:null;
  const rd=s.rampDemand-snap.rd,bl=s.rampBlocked-snap.bl;
  const dem=s.entered-snap.en+(s.queue-snap.q); // main demand arrivals in window ≈ entered + change in virtual queue
  const m={over,seed:sd,opt,veh:ex/Wh,ppl:pp/Wh,rampOut:mg/Wh,mainOut:(ex-mg)/Wh,
    rampBlkPct:rd?100*bl/rd:0,rampWait:(s.merged-snap.mg)?(s.waitSum-snap.ws)/(s.merged-snap.mg):null,
    qEnd:s.queue,qGrow:(s.queue-snap.q)/Wh,
    trip:tripMean,tripV,vmt:vm,vmtPerH:vm/Wh,
    tetPerKvm:1000*tet/Math.max(vm,1),hb9PerKvm:1000*hb9/Math.max(vm,1),hb15PerKvm:1000*hb15/Math.max(vm,1),
    lcPerKvm:1000*(s.lcCount-snap.lc)/Math.max(vm,1),
    nMerge:mergeRecs.length,mergeTight:mergeRecs.filter(r=>(r.fg!=null&&r.fg<20)||(r.fttc!=null&&r.fttc<2)).length,
    mergeForced:mergeRecs.filter(r=>r.forced).length,
    mergeMinFg:mergeRecs.length?Math.min(...mergeRecs.map(r=>r.fg==null?1e9:r.fg)):null,
    nBD:s.bdEvents.length,crashInfo,
    cong:(()=>{const thr=0.6*P.mean;const A=s.detLog.filter(r=>r.sensor==='A'&&r.lane==='R'&&r.t1>WU);return A.length?A.filter(r=>(r.smean_mph==null?0:r.smean_mph)<thr).length/A.length:null})()};
  return m;
}
module.exports={run2};
if(require.main===module){
  const jobs=JSON.parse(fs.readFileSync(process.argv[2]));const out=process.argv[3];
  for(const j of jobs){const r=run2(j.over,j.seed,j.opt||{});r.tag=j.tag;fs.appendFileSync(out,JSON.stringify(r)+'\n')}
}
