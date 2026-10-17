/* 仅电脑端生成；不接串口、调试器或车辆。 */
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const base = path.resolve(__dirname, '../lookup');
if (!process.argv[2]) throw Error('Pass the compiled map export JSON from rebuild-route-table.ps1');
const mapInput = JSON.parse(fs.readFileSync(process.argv[2], 'utf8').replace(/^\uFEFF/, ''));
if (!mapInput.geometryValidationPassed) throw Error('Map geometry was not validated');
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const roads = mapInput.roads;
if (roads.length !== 12 || roads.some(([a,b,l]) => a >= 9 || b >= 9 || l <= 0))
  throw Error('Expected the current nine-node/twelve-road map');
const adj = Array.from({length:9}, () => []);
roads.forEach(([a,b,length],i) => {
  adj[a].push({node:b,length,bit:1<<i});
  adj[b].push({node:a,length,bit:1<<i});
});

// 与现有 route.c 同样的节点扫描和严格松弛顺序；正式等价性另由 C 验证器核对。
function baseline(from,to,blocked,passed) {
  const d = Array(9).fill(65535), u = Array(9).fill(65535);
  const prev = Array(9).fill(255), done = Array(9).fill(false);
  d[from] = u[from] = 0;
  for (let k=0;k<9;k++) {
    let best = -1;
    for (let i=0;i<9;i++) if (!done[i] && (best<0 || d[i]<d[best] ||
      d[i]===d[best] && u[i]<u[best])) best=i;
    if (best<0 || d[best]===65535) break;
    done[best]=true;
    if (best===to) break;
    roads.forEach(([a,b,l],i) => {
      if (blocked & (1<<i) || a!==best && b!==best) return;
      const n=a===best?b:a, nd=d[best]+l, nu=u[best]+(passed & (1<<i)?0:l);
      if (nd<d[n] || nd===d[n] && nu<u[n]) {d[n]=nd;u[n]=nu;prev[n]=best;}
    });
  }
  if (d[to]===65535) return null;
  const nodes=[to];
  while(nodes[nodes.length-1]!==from) nodes.push(prev[nodes[nodes.length-1]]);
  return {distance:d[to],unknown:u[to],nodes:nodes.reverse()};
}

const pairs=[];
for (let from=0;from<9;from++) for(let to=0;to<9;to++) {
  const paths=[];
  function visit(n,seen,mask,length,nodes) {
    if(n===to) {paths.push({mask,length,nodes});return;}
    for(const e of adj[n]) if(!(seen & (1<<e.node)))
      visit(e.node,seen|(1<<e.node),mask|e.bit,length+e.length,[...nodes,e.node]);
  }
  visit(from,1<<from,0,0,[from]);
  paths.sort((a,b)=>a.length-b.length);
  if(paths.length>16) throw Error('Direct path codes overflow');
  pairs.push({from,to,paths,classes:[],keys:new Map()});
}

const primary=Buffer.alloc(4096*81,255);
let tiedQueries=0;
for(let blocked=0;blocked<4096;blocked++) for(let pair=0;pair<81;pair++) {
  const p=pairs[pair], valid=[];
  let length=Infinity;
  for(let i=0;i<p.paths.length;i++) {
    const r=p.paths[i];
    if(r.mask & blocked) continue;
    if(r.length>length) break;
    length=r.length;valid.push(i);
  }
  if(valid.length===1) primary[blocked*81+pair]=valid[0];
  else if(valid.length>1) {
    tiedQueries++;
    const key=valid.join(',');
    if(!p.keys.has(key)) {
      let union=0,common=4095;
      for(const i of valid) {union|=p.paths[i].mask;common&=p.paths[i].mask;}
      p.keys.set(key,p.classes.length);
      p.classes.push({valid,blocked,mask:union & ~common});
    }
    const code=16+p.keys.get(key);
    if(code>=255) throw Error('Decision class codes overflow');
    primary[blocked*81+pair]=code;
  }
}

const secondary=[],decisions=[],decisionBase=[],pathBase=[],flatPaths=[];
for(const p of pairs) {
  pathBase.push(flatPaths.length);flatPaths.push(...p.paths);
  decisionBase.push(decisions.length);
  for(const c of p.classes) {
    const bits=Array.from({length:12},(_,i)=>i).filter(i=>c.mask & (1<<i));
    const offset=secondary.length;
    for(let packed=0;packed<2**bits.length;packed++) {
      let passed=0;
      bits.forEach((b,i)=>{if(packed & (1<<i)) passed|=1<<b;});
      const r=baseline(p.from,p.to,c.blocked,passed);
      const i=p.paths.findIndex(v=>v.nodes.join(',')===r.nodes.join(','));
      if(!c.valid.includes(i)) throw Error('Generated decision is not a shortest candidate');
      secondary.push(i);
    }
    decisions.push({mask:c.mask,offset});
  }
}
function array(type,name,values,format=String) {
  const lines=[];
  for(let i=0;i<values.length;i+=32) lines.push('    '+values.slice(i,i+32).map(format).join(','));
  return `static const ${type} ${name}[${values.length}]={\n${lines.join(',\n')}\n};\n`;
}
const header='/* 自动生成：tools/rebuild-route-table.ps1；禁止手工修改。 */\n'+
  '#ifndef NINE_TABLE_DATA_H\n#define NINE_TABLE_DATA_H\n'+
  `#define TABLE_MAP_TOP_INSET_MM ${mapInput.parameters.topInsetMm}\n`+
  `#define TABLE_MAP_BOTTOM_INSET_MM ${mapInput.parameters.bottomInsetMm}\n`+
  `#define TABLE_MAP_RAW_EXTRA_INSET_MM ${mapInput.parameters.rawExtraInsetMm}\n`+
  `#define TABLE_MAP_STORAGE_INSET_MM ${mapInput.parameters.storageInsetMm}\n`+
  'typedef struct { uint32_t offset; uint16_t mask; } TableDecision;\n'+
  array('uint8_t','table_primary',[...primary])+
  array('uint8_t','table_secondary',secondary)+
  array('uint16_t','table_path_base',pathBase)+
  array('uint16_t','table_decision_base',decisionBase)+
  array('TableDecision','table_decisions',decisions,d=>`{${d.offset}u,${d.mask}u}`)+
  array('RouteTablePath','table_paths',flatPaths,p=>`{${p.length}u,${p.nodes.length},{${p.nodes.concat(Array(9-p.nodes.length).fill(255)).join(',')}}}`)+
  '#endif\n';
fs.writeFileSync(path.join(base,'table_data.h'),header);
const record={
  schemaVersion:1,generatedAt:new Date().toISOString(),
  rule:'distance, then unknown distance, then exactly the current C baseline route',
  sourceHashes:mapInput.sourceHashes, parameters:mapInput.parameters, nodes:mapInput.nodes,
  roads,states:3**12,nodePairs:81,queries:3**12*81,
  primaryBytes:primary.length,secondaryBytes:secondary.length,
  paths:flatPaths.length,decisionClasses:decisions.length,
  maxClassesPerPair:Math.max(...pairs.map(p=>p.classes.length)),tiedBlockedQueries:tiedQueries,
  tableDataSha256:sha(path.join(base,'table_data.h')),
  validation:'pending; reduced passed masks and shared decision classes must pass exhaustive C verification'
};
fs.writeFileSync(path.join(base,'generation.json'),JSON.stringify(record,null,2)+'\n');
console.log(JSON.stringify(record,null,2));
