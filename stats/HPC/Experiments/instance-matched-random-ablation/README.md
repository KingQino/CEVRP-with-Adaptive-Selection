# Instance-Matched Random Ablation

This branch compares the No-Elite contextual online policy with a conservative
oracle random control. For each instance, the control receives the online
policy's aggregate weak, medium, and bounded-strong decision proportions from
the existing 20 No-Elite runs. Individuals that reached a local optimum during
the common weak probe are excluded when estimating and applying these ratios.

The control receives no context and performs no learner updates. Within each
generation, it keeps cumulative action counts within one decision of the
instance target and randomly permutes the assignments within each workspace
batch. Thus, it preserves the instance-level marginal action mix without
copying which individual or search state received each action.

The committed ratios are in
`config/instance-matched-random-ratios.tsv`. They can be regenerated with:

```bash
python3 scripts/generate_instance_matched_ratios.py
```

On Apocrita, run from the repository root:

```bash
bash stats/HPC/Experiments/instance-matched-random-ablation/setup.sh
```

After all array jobs finish:

```bash
bash stats/HPC/Experiments/instance-matched-random-ablation/summarize.sh
```
