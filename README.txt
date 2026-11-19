Quick check (needs Node.js 18+):
  node run2.js job.json out.jsonl
Then compare out.jsonl with the matching line in res.jsonl
(tag "E1", over {"flow":3000,"ramp":200}, seed 1). Values should match exactly.

Full experiments: node run2.js jobs_new.json my_results.jsonl  (about 15 min)
Metering runs:    node run2.js jobs_e6.json my_e6.jsonl
