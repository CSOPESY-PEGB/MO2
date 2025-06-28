## Group Members:

---

S13: {Joel Ethan Batac, Joshua Gilo}</br>
S14: {Paul Ivan Enclonar, Peter Parker}


ENTRY POINT
-----------
The main function serves as the entry point of the program. <br>
```-> main.cpp```

PREREQUISITES
-------------
To build and run this program, you will need:

1.  A modern C++ compiler that supports the C++20 standard.
  -   GCC 10 or newer
  -   Clang 12 or newer
  -   MSVC v19.29 (Visual Studio 2019) or newer

2.  CMake version 3.16 or newer.


BUILD & RUN INSTRUCTIONS
------------------------
The project is organized with header files in `include/`, source files in `src/`, and is built using CMake. Follow these steps from your terminal or command prompt.

1.  **File Setup:**
    Ensure your directory structure looks like this:

    csopesy/ <br>
    ├── CMakeLists.txt<br>
    ├── include/<br>
    │   ├── commands.hpp<br>
    │   └── ... (all other .hpp files)
    └── src/<br>
    ├── commands.cpp<br>
    └── ... (all other .cpp files)<br><br>

2.  **Navigate to Project Root:**
    Open your terminal and change your directory to the root `csopesy` folder.

    cd path/to/csopesy

3.  **Create a Build Directory:**
    It is standard practice to perform an "out-of-source" build. This keeps the project directory clean.

    mkdir build
    cd build

4.  **Configure the Project:**
    Run CMake from the `build` directory to generate the build files for your system (e.g., Makefiles on Linux/macOS, Visual Studio solution on Windows). The `..` tells CMake to look for the `CMakeLists.txt` in the parent directory.

    cmake ..

5.  **Compile the Project:**
    Run the build command. This will compile all the source code and link it into a single executable named `sim`.

    cmake --build .

    (Alternatively, on Linux/macOS, you can just run `make` after the `cmake ..` step).

   6.  **Run the Emulator:**
       After a successful build, the executable `sim` (or `sim.exe` on Windows) will be located in the `build` directory.

       To run it, execute the following command from inside the `build` directory:

       ./sim

       **Note:** The emulator will look for `config.txt` in the directory you run it from (the `build` directory).<br>
       Make sure you have a `config.txt` file there before running the `initialize` command.<br>



BASIC USAGE
-----------
Once the emulator is running, you will see the `~` prompt.

1.  **Initialize the System:**
    The first command you should run is `initialize`. This will load the configuration from `config.txt` (or create a default one). Make sure a `config.txt` file exists in the directory where you run the executable (the `build` directory).
    Alternatively, you may try running this command `initialize ../config.txt`<br>

    `~ initialize` <br>

2.  **Start Generating Processes:**
    To have the scheduler automatically create and run processes based on the config, use:

    `~ scheduler-start` <br> 
    `~ screen -s <process_name>`<br>
    
    To stop generation of processes run this command `~ scheduler-stop`<br>
<br>

3.  **View System Status:**
    To see a list of running and finished processes, and check CPU utilization, use the `screen` command with the `-ls` flag:

    `~ screen -ls` <br>
    
    To view the logs inside a process do: `~ screen -r <process_name>` <br>
        

4.  **Exit the Emulator:**
    To shut down the emulator cleanly, type:

    `~ exit`