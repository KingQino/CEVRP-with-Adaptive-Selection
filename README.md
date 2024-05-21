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

