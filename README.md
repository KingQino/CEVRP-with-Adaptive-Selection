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

2. Second step - run

   ```shell
   ./Run E-n22-k4.evrp 1 1 strong
   
   # Explanation
   # ./Run <instance> <stop_criteria> <multithreading> [weak|medium|strong]
   ```
   



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
- `Leader` retains the full-descent LS-3/5/7 variants as baselines. LS-7-RVND-OneMove supports interruptible weak, medium, and strong intensities; each selected neighborhood accepts at most one improving move and no empty-route move is used. Strong remains the default.
- Weak and medium cap accepted moves at 2% and 10% of `customer_count + route_count`; strong runs until all seven neighborhoods fail.
- Every generation applies the selected local-search intensity to the complete upper-level population; no confidence filter is used.
- Lower-level charging is evaluated only for solutions within `1.02 * global_best_upper_cost`.
- Each trial writes per-call local-search feedback to `local-search.tsv` in its stats directory.
- `Follower` inserts charging stations and refines a complete solution by enumeration.
- `Reproduction` rebuilds the complete population with 85% upper-parent offspring, 5% verified-best/immigrant offspring, and 10% pure immigrants; no lower-level elite is injected directly.
