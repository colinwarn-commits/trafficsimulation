const {run} = require('./run.js');
const fs = require('fs');
const jobs = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
for (const j of jobs) {
  const r = run(j.over, j.seed, j.dur || 3600);
  fs.appendFileSync(process.argv[3], JSON.stringify(r) + '\n');
}
