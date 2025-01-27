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

   ```sh
   git clone -b no-logging git@github.com:KingQino/CEVRP-with-Adaptive-Selection.git
   ```

   ```shell
   ml load cmake/3.23.1 gcc/12.1.0 openmpi/4.1.4-gcc
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


3. Sulis 

   ```sh
   ml load CMake/3.18.4 GCC/13.2.0
   sbatch script.slurm 
   ```

   `./build/script.slurm`

   ```sh
   #!/bin/bash
   
   # Slurm job options (job-name, compute nodes, job time)
   #SBATCH --job-name=CEVRP-with-Adaptive-Selection
   #SBATCH --output=path/log/slurm-%A_%a.out
   #SBATCH --time=48:0:0                   # Request 48 hours of compute time
   #SBATCH --nodes=1                       # Request 1 node
   #SBATCH --tasks-per-node=1              # One task per node
   #SBATCH --cpus-per-task=10              # Each task uses 10 CPUs (threads)
   #SBATCH --mem-per-cpu=1G                # Memory per CPU
   #SBATCH --account=su008-exx866
   #SBATCH --array=0-16
   
   module load GCC/13.2.0
   
   mapfile -t cases < parameters.txt
   CASE="${cases[$SLURM_ARRAY_TASK_ID]}"
   
   srun ./Run "$CASE" 2 1
   ```

   `./build/parameters.txt`

   ```sh
   E-n22-k4.evrp
   E-n23-k3.evrp
   E-n30-k3.evrp
   E-n33-k4.evrp
   E-n51-k5.evrp
   E-n76-k7.evrp
   E-n101-k8.evrp
   X-n143-k7.evrp
   X-n214-k11.evrp
   X-n351-k40.evrp
   X-n459-k26.evrp
   X-n573-k30.evrp
   X-n685-k75.evrp
   X-n749-k98.evrp
   X-n819-k171.evrp
   X-n916-k207.evrp
   X-n1001-k43.evrp
   ```

   `setup.sh`

   ```sh
   #!/bin/bash
   
   # Target directories
   directories=$(ls -d */ | grep -v '^CBMA-/')
   
   # Check if any directories were found
   if [ -z "$directories" ]; then
       echo "No directories found."
       exit 1
   fi
   
   # Loop through each directory to set up build structure and create script.slurm
   for dir in $directories; do
       # Remove trailing slash from directory name
       dir=${dir%/}
   
       # Check if the directory exists
       if [ -d "$dir" ]; then
           # Define paths for build and log folders
           build_dir="$dir/build"
           log_dir="$build_dir/log"
   
           # Create the build and log directories if they don't exist
           mkdir -p "$log_dir"
           echo "Created '$log_dir'."
   
           # Copy parameters.txt into the build folder
           if [ -f "parameters.txt" ]; then
               cp parameters.txt "$build_dir"
               echo "Copied 'parameters.txt' to '$build_dir'."
           else
               echo "'parameters.txt' not found in the current directory."
           fi
   
           # Create script.slurm with dynamic content
           cat > "$build_dir/script.slurm" <<EOL
   #!/bin/bash
   
   # Slurm job options (job-name, compute nodes, job time)
   #SBATCH --job-name=$dir                             # Job name set to the parent directory name
   #SBATCH --output=$(pwd)/$log_dir/slurm-%A_%a.out       # Output log file path in the log folder
   #SBATCH --time=48:0:0                   # Request 48 hours of compute time
   #SBATCH --nodes=1                       # Request 1 node
   #SBATCH --tasks-per-node=1              # One task per node
   #SBATCH --cpus-per-task=20              # Each task uses 10 CPUs (threads)
   #SBATCH --mem-per-cpu=1G                # Memory per CPU
   #SBATCH --account=su008-exx866
   #SBATCH --array=0-16
   
   module load GCC/13.2.0
   
   mapfile -t cases < "parameters.txt"        # Load parameters.txt from the build directory
   CASE="\${cases[\$SLURM_ARRAY_TASK_ID]}"
   
   srun ./Run "\$CASE" 2 1
   EOL
   
           echo "Generated 'script.sh' in '$build_dir'."
   
           # Navigate to build_dir, run cmake and make commands
           (
               cd "$build_dir" || exit
               cmake ..
               make
               sbatch script.slurm
           )
       else
           echo "Directory '$dir' does not exist."
       fi
   done
   ```

   - For **Apocrita**

     load module

     ```sh
     ml load cmake/3.23.1 gcc/12.1.0 openmpi/4.1.4-gcc
     ```

     `setup.sh`

     ```sh
     #!/bin/bash
     
     # Target directories
     directories=$(ls -d */ | grep -Ev '^(CBMA-CEC|CBMA-CEC-OPM|CEVRP-with-Adaptive-Selection)/')
     
     # Check if any directories were found
     if [ -z "$directories" ]; then
         echo "No directories found."
         exit 1
     fi
     
     # Loop through each directory to set up build structure and create script.slurm
     for dir in $directories; do
         # Remove trailing slash from directory name
         dir=${dir%/}
     
         # Check if the directory exists
         if [ -d "$dir" ]; then
             # Define paths for build and log folders
             build_dir="$dir/build"
             log_dir="$build_dir/log"
     
             # Create the build and log directories if they don't exist
             mkdir -p "$log_dir"
             echo "Created '$log_dir'."
     
             # Copy parameters.txt into the build folder
             if [ -f "parameters.txt" ]; then
                 cp parameters.txt "$build_dir"
                 echo "Copied 'parameters.txt' to '$build_dir'."
             else
                 echo "'parameters.txt' not found in the current directory."
             fi
     
             # Create script.slurm with dynamic content
             cat > "$build_dir/script.sh" <<EOL
     #!/bin/bash
     #$ -pe smp 10
     #$ -l h_vmem=1G
     #$ -l h_rt=48:0:0
     #$ -cwd
     #$ -j y
     #$ -N $dir
     #$ -o $(pwd)/$log_dir
     #$ -t 1-17
     
     module load gcc/12.1.0
     
     set -e
     
     mapfile -t cases < parameters.txt
     CASE="\${cases[\$((SGE_TASK_ID - 1))]}"
     
     # Check if CASE is valid
     if [ -z "\$CASE" ]; then
         echo "Error: No parameter found for SGE_TASK_ID=\$SGE_TASK_ID" >&2
         exit 1
     fi
     
     echo "Running task \$SGE_TASK_ID with CASE=\$CASE"
     
     ./Run "\$CASE" 2 1
     EOL
     
             echo "Generated 'script.sh' in '$build_dir'."
     
             # Navigate to build_dir, run cmake and make commands
             (
                 cd "$build_dir" || exit
                 cmake ..
                 make
                 qsub script.sh
             )
         else
             echo "Directory '$dir' does not exist."
         fi
     done
     ```

     

4. Experimental results

   `objective.sh`

   ```sh
   #!/bin/bash
   
   output_file="a.txt"
   > "$output_file" # Clear or create the output file
   
   # List of directories and their specific stats files in the desired order
   declare -A stats_files=(
       [E-n22-k4]="stats.E-n22-k4.evrp"
       [E-n23-k3]="stats.E-n23-k3.evrp"
       [E-n30-k3]="stats.E-n30-k3.evrp"
       [E-n33-k4]="stats.E-n33-k4.evrp"
       [E-n51-k5]="stats.E-n51-k5.evrp"
       [E-n76-k7]="stats.E-n76-k7.evrp"
       [E-n101-k8]="stats.E-n101-k8.evrp"
       [X-n143-k7]="stats.X-n143-k7.evrp"
       [X-n214-k11]="stats.X-n214-k11.evrp"
       [X-n351-k40]="stats.X-n351-k40.evrp"
       [X-n459-k26]="stats.X-n459-k26.evrp"
       [X-n573-k30]="stats.X-n573-k30.evrp"
       [X-n685-k75]="stats.X-n685-k75.evrp"
       [X-n749-k98]="stats.X-n749-k98.evrp"
       [X-n819-k171]="stats.X-n819-k171.evrp"
       [X-n916-k207]="stats.X-n916-k207.evrp"
       [X-n1001-k43]="stats.X-n1001-k43.evrp"
   )
   
   # Process files in the given sequence
   for dir in E-n22-k4 E-n23-k3 E-n30-k3 E-n33-k4 E-n51-k5 E-n76-k7 E-n101-k8 \
              X-n143-k7 X-n214-k11 X-n351-k40 X-n459-k26 X-n573-k30 X-n685-k75 \
              X-n749-k98 X-n819-k171 X-n916-k207 X-n1001-k43; do
       file_path="$dir/${stats_files[$dir]}"
       if [ -f "$file_path" ]; then
           tail -n 3 "$file_path" >> "$output_file"
       else
           echo "File not found: $file_path" >&2
       fi
   done
   
   # Process the output file to ensure results follow the sequence
   awk '
   /Mean/ {
       mean_value = $2;
       std_dev_value = $NF;
   }
   /Min:/ {
       min_value = $2;
       print min_value;
       print mean_value;
       print std_dev_value;
   }' "$output_file"
   
   # Uncomment the following line if you want to delete a.txt after processing
   rm -f "$output_file"
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

