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
   ./Run E-n22-k4.evrp 1 1
   
   # Explanation
   # ./Run <problem_instance_filename> <stop_criteria: 1 for max-evals, 2 for max-time> <multithreading: 1 for yes>
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
- `Leader` retains the full-descent LS-3/5/7 variants as baselines. The default LS-7-RVND-OneMove uses the same seven neighborhoods, but each selected neighborhood accepts at most one improving move before returning to RVND; it does not use empty-route moves.
- `Follower` inserts charging stations and refines a complete solution by enumeration.
- `Reproduction` owns population ranking, the depot-aware quality-diversity parent pool, genetic operators, and the 85%/5%/10% offspring strategy.
