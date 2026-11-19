import json, collections

d = collections.defaultdict(list)
for line in open('sw1.jsonl'):
    r = json.loads(line)
    d[r['over']['flow']].append(r)

agg = {}
for q, rs in sorted(d.items()):
    g = lambda k: [r[k] for r in rs if r[k] is not None]
    agg[q] = dict(n=len(rs), cong=g('congFrac'), nbd=g('nBD'), dq=g('dq'),
                  aRu=g('aRu'), trip=g('trip'), tripV=g('tripV'),
                  wait=g('rampWait'), lc=g('lc'))

json.dump(agg, open('agg1.json', 'w'))
print('wrote agg1.json')
