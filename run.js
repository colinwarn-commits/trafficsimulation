require('./sim.js');
const {P,reset,step,S,idmEq,consts}=__sim;
const DEF=JSON.parse(JSON.stringify(P));
function avg(a){return a.length?a.reduce((x,y)=>x+y,0)/a.length:null}
function run(over,sd,dur=3600,keep=false){
  Object.assign(P,DEF,{crash:0},over);seed(sd);reset();
  const n=Math.round(dur/0.1);for(let i=0;i<n;i++)step();
  const s=S();const thr=0.6*P.mean;
  const ints={};for(const r of s.detLog){const k=r.t1;(ints[k]=ints[k]||[]).push(r)}
  const keys=Object.keys(ints).map(Number).sort((a,b)=>a-b);
  const rows=[];
  for(const k of keys){const g=ints[k];const aR=g.find(r=>r.sensor==='A'&&r.lane==='R'),aL=g.find(r=>r.sensor==='A'&&r.lane==='L'),dR=g.find(r=>r.sensor==='D'&&r.lane==='R'),dL=g.find(r=>r.sensor==='D'&&r.lane==='L'),B=g.find(r=>r.sensor==='B'),C=g.find(r=>r.sensor==='C');
    rows.push({t:k,aRu:aR.smean_mph,aLu:aL.smean_mph,aRq:aR.flow_vph,aLq:aL.flow_vph,aRk:aR.density_vpm,aLk:aL.density_vpm,aRocc:aR.occ_pct,aLocc:aL.occ_pct,dq:dR.flow_vph+dL.flow_vph,dRq:dR.flow_vph,dLq:dL.flow_vph,dRu:dR.smean_mph,dLu:dL.smean_mph,dRk:dR.density_vpm,dLk:dL.density_vpm,dRocc:dR.occ_pct,bq:B.flow_vph,blk:B.blocked,cq:C.flow_vph,cavgq:C.avg_queue,cmax:C.max_queue})}
  const post=rows.filter(r=>r.t>600);
  const uA=r=>r.aRu==null?0:r.aRu;
  const cong=post.filter(r=>uA(r)<thr).length;
  const m={over,seed:sd,nBD:s.bdEvents.length,congFrac:cong/post.length,
    dq:avg(post.map(r=>r.dq)),aRu:avg(post.map(uA)),aRq:avg(post.map(r=>r.aRq)),aLq:avg(post.map(r=>r.aLq)),
    ev:s.bdEvents.map(e=>({t:e.t,end:e.end,pre:e.pre,qdf:e.qdf,drop:e.drop,w:e.w})),
    blockedFrac:s.rampDemand?s.rampBlocked/s.rampDemand:0,merged:s.merged,rampWait:s.merged?s.waitSum/s.merged:null,
    entered:s.entered,exited:s.exited,trip:s.tripN?s.tripSum/s.tripN:null,tripV:s.tripSum?s.tripD/s.tripSum/consts.MPH:null,lc:s.lcCount,queueEnd:s.queue,
    maxRampQ:Math.max(...post.map(r=>r.cmax)),avgRampQ:avg(post.map(r=>r.cavgq)),wave:s.wave};
  if(keep)return {m,rows,s};
  return m;
}
module.exports={run,P,DEF,idmEq,consts,S};
if(require.main===module){
  const job=JSON.parse(process.argv[2]);const out=[];
  for(const j of job){const r=run(j.over,j.seed,j.dur||3600);require('fs').appendFileSync(process.argv[3],JSON.stringify(r)+'\n');}
}
