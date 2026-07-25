## Paper

For details, please refer to the following paper:
```
@inproceedings{qin2024confidence,
  title={A Confidence-based Bilevel Memetic Algorithm with Adaptive Selection Scheme for Capacitated Electric Vehicle Routing Problem},
  author={Qin, Yinghao and Chen, Jun},
  booktitle={2024 IEEE Congress on Evolutionary Computation (CEC)},
  pages={1--10},
  year={2024},
  organization={IEEE}
}
```

## Usage

1. First step - compile

   ```shell
   mkdir build
   cd build
   cmake ..
   make
   ```

2. Second step - run one seeded trial without logs

   ```shell
   ./Run -ins E-n22-k4.evrp -seed 1 -stp 1 -mth 0 -log 0 -ls strong
   ```

   The command prints only the final `lower_cost`, which can be consumed directly
   by IRACE. Set `-mth 1` to run ten trials in parallel and print their mean, or
   `-log 1` to write evolution, solution, and aggregate local-search logs.

   The former positional syntax remains available:

   ```shell
   ./Run E-n22-k4.evrp 1 0 strong
   ```

### Tunable Parameters

| Option | Default | Meaning |
| --- | ---: | --- |
| `-pop_size` | `100` | Population size |
| `-mutation_prob` | `0.5` | Probability that an offspring enters mutation |
| `-mutation_ind_prob` | `0.2` | Per-gene probability inside the mutation operator |
| `-tournament_size` | `2` | Upper-parent tournament size |
| `-ls` | `strong` | `skip`, `weak`, `medium`, `bounded_strong`, or `strong` local-search intensity |
| `-ls_policy` | `static` | Local-search allocation policy: `static`, `random`, `matched_random`, `non_contextual`, or `online` |
| `-parent_pool_ratio` | `0.10` | Parent-pool size relative to population size |
| `-quality_ratio` | `0.50` | Quality-selected share of the parent pool |
| `-verified_upper_ratio` | `0.05` | `verifiedBest x P_upper` offspring share |
| `-pure_immigrant_ratio` | `0.10` | Pure immigrant offspring share |
| `-gamma` | `1.02` | Upper-cost ratio that triggers follower evaluation |

Invalid configurations exit with a non-zero status. Probabilities must be in
`[0, 1]`, `quality_ratio` must be strictly between 0 and 1,
`parent_pool_ratio * pop_size >= 5`, and
`verified_upper_ratio + pure_immigrant_ratio <= 0.25`. The tournament size
cannot exceed the resulting parent-pool size.

`random` uses the original progressive random mix. `matched_random` assigns
approximately 62% weak, 9% medium, and 29% deepest actions independently of
context and feedback. `non_contextual` uses the online learner with a constant
context, while `online` includes the per-individual search context. The deepest
action is selected by `-ls strong` or `-ls bounded_strong`.



## Project Structure

```
.
├── CMakeLists.txt
├── README.md
├── LICENSE
├── data
│   ├── ...
│   └── X-n916-k207.evrp
├── include
│   ├── MA.hpp
│   ├── algorithm_constants.hpp
│   ├── case.hpp
│   ├── follower.hpp
│   ├── individual.hpp
│   ├── initializer.hpp
│   ├── leader.hpp
│   ├── reproduction.hpp
│   ├── stats.hpp
│   └── ...
├── src
│   ├── MA.cpp
│   ├── case.cpp
│   ├── follower.cpp
│   ├── individual.cpp
│   ├── initializer.cpp
│   ├── leader.cpp
│   ├── reproduction.cpp
│   ├── stats.cpp
│   └── ...
└── main.cpp

```

> - `data`: instance files
> - `include`: header files
> - `src`: source files

## Algorithm Modules

- `MA` coordinates one generation and owns the population, stopping criteria, and logging.
- `Initializer` builds capacity-feasible upper-level routes with clustering, random split, or direct encoding.
- `Leader` retains the full-descent LS-3/5/7 and LS-7-RVND-OneMove variants as baselines. The main search uses LS-8-RVND-OneMove, adding SWAP* with exact top-3 insertion caches to evaluate best feasible reinsertion positions in quadratic time per route pair. It supports skip, weak, medium, bounded-strong, and strong intensities; each selected neighborhood accepts at most one improving move and no empty-route move is used. Strong remains the default.
- Skip performs no upper-level local search, while weak and medium cap accepted moves at 2% and 10% of the initial `customer_count + route_count`. Bounded-strong first performs the weak probe in the same RVND session, then continues up to 30% of that initial scale and a soft cumulative limit of 128 times the weak probe's distance calls. Unlimited strong runs until all eight neighborhoods fail. Gamma filtering and follower evaluation still run after skip.
- The default `static` policy applies the selected intensity to the complete upper-level population. The `random` policy retains the fixed terminal mix of approximately 20% weak, 30% medium, and 50% strong as an ablation baseline.
- The `online` policy gives every new individual a weak probe, then independently chooses whether it terminates at weak, medium, or the deepest intensity selected by `-ls`. Use `-ls bounded_strong` to learn among weak/medium/bounded-strong, or `-ls strong` to retain weak/medium/unlimited-strong. It has no fixed action quota. Context contains only changing search, individual, and probe feedback; instance size and route-count features are intentionally excluded because the model is reset for every run.
- Allocated policies preserve each individual's RVND session and failure cache while progressing from weak to medium or strong. Online reward combines lower-archive rank, post-probe gamma crossing, reproduction parent usage, and continuation upper gain. Action cost contains only evaluations consumed after the common weak probe.
- All policies maintain the complete historical best upper-level individual as the shared context reference and final fallback.
- Lower-level charging is evaluated only for solutions within `1.02 * global_best_upper_cost`.
- All policies write eight per-generation operator totals to `local-search-operators.tsv`. Random and online allocation additionally write three action totals per generation to `local-search-allocation.tsv`; per-individual local-search rows are intentionally omitted to keep long runs compact.
- `Follower` inserts charging stations and refines a complete solution by enumeration.
- With `popSize=100`, `Reproduction` builds the next generation from one lower-level elite, 84 upper-parent offspring, 5 `verifiedBest x P_upper` offspring, and 10 pure immigrants.
