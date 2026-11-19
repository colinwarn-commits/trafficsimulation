const {run,P,idmEq,consts}=require('./run.js');
const fs=require('fs');
function dump(name,over,sd,dur){
  const {m,rows,s}=run(over,sd,dur,true);
  const t=s.t,t0=t-900;
  const trs=[];
  const add=(L,p)=>{const q=p.filter(a=>a[0]>=t0);if(q.length>1)trs.push({L,p:q.map(a=>[+a[0].toFixed(1),+(a[1]/consts.M2F).toFixed(1)])})};
  for(const s2 of s.trajStore)add(s2.L,s2.p);
  for(let L=0;L<2;L++)for(const c of s.lanes[L])add(L,c.tr);
  const col=s.lastCol,NT=consts.NT,NX=consts.NX;
  const sf=[[],[]];
  for(let L=0;L<2;L++){for(let a=col-NT+1;a<=col;a++){if(a<0)continue;const r=(a%NT)*NX,row=[];for(let x=0;x<NX;x++){const c=s.sfCnt[L][r+x];row.push(c?+(s.sfSum[L][r+x]/c).toFixed(1):null)}sf[L].push({a,row})}}
  fs.writeFileSync(name+'.json',JSON.stringify({m,rows,trs,sf,gen:s.gen,act:s.act,actN:s.actN,t,col,bd:s.bdEvents,wave:s.wave,P:{...P},eq:idmEq()}));
}
dump('runA',{flow:2000,ramp:200},1,3600);
dump('runB',{flow:3000,ramp:200},3,3600);
dump('runC',{flow:2000,ramp:900,nmax:20},1,3600);
