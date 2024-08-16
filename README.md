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

## Usage :dog:

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
   



## Project Structure :dolphin:

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
│   ├── case.hpp
│   ├── heuristic.hpp
│   ├── individual.hpp
│   ├── stats.hpp
│   └── utils.hpp
├── src
│   ├── MA.cpp
│   ├── case.cpp
│   ├── heuristic.cpp
│   ├── individual.cpp
│   ├── stats.cpp
│   └── utils.cpp
└── main.cpp

```

> - `data`: instance files
> - `include`: header files
> - `src`: source files

