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


## Experiments :deer:

- Algorithms Comparison :cactus:

  | VNS  | SA   | GSGA | BACO | CBACO-D | CBACO-I | CBMA |
  | ---- | ---- | ---- | ---- | ------- | ------- | ---- |
  
  > Codes:
  >
  > - [VNS](https://github.com/wolledav/VNS-EVRP-2020)
  > - [GSGA](https://github.com/NeiH4207/EVRP) 
  > - [BACO & CBACO-D & CBACO-I](https://github.com/Flyki/CEVRP)
  > - SA: no paper, no code

* Adaptive selection scheme :vs: typical tournament selection

  | CBMA with customized selection | CBMA with tournament selection |
  | ------------------------------ | ------------------------------ |

  > CBMA with tournament selection $\Rightarrow$ premature convergence
  >
  > CBMA with adpative selection scheme $\Rightarrow$ strong exploration capacity

* CBMA :vs: BMA

  | CBMA | BMA  |
  | ---- | ---- |

  > Compare the states of BMA with and without confidence-based selection strategy. 
  >
  > - BMA  $\Rightarrow$ low efficiencent
  > - CBMA $\Rightarrow$ high efficiencent

* Ablation Study (We didn't conduct this experiment due to paper max-page-number limitation)

    1. local search operators

       | baseline | remove 2-opt | remove 2-opt* | remove node-shift |
       | -------- | ------------ | ------------- | ----------------- |
       | top-1    | top-2        | top-4         | top-3             |

       | baseline | 2-opt | 2-opt* | node-shift |
       | -------- | ----- | ------ | ---------- |

    2. population initialization (0ptional)

    3. refine (optional)

