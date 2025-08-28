# Project setup
1. Download or clone via git this repository;
2. Install one of the following IDE:
   - Visual studio code (https://code.visualstudio.com/)
   - CLion jetbrains (https://www.jetbrains.com/clion/)
3. Follow the guide for the Agent Setup;
4. Download or clone the simulation environment (https://github.com/tonegas/PyDrivingSim);
5. Follow the guide for the evirorment setup;

## Agent Setup
### Window
#### Setup using WSL:
1. Follow the guide to install wsl
   1. https://learn.microsoft.com/it-it/windows/wsl/install
2. Open a Linux shell and run:
```bash
sudo apt-get update
sudo apt-get install cmake
sudo apt-get install clang
```
3. Copy from the folder lib/win_WLS the communication library to the folder lib/
4. Compile the agent:
   - In Clion:
      1. Open the folder project and create Debug and Release profile
      2. Select the correct compiler WLS (follow the guide https://www.jetbrains.com/help/clion/how-to-create-toolchain-in-clion.html)
      3. Compile the agent
   
   - In a wsl terminal
      1. Compile the agent using the following command:
         ```bash
         cmake .
         make
         ```
5. Run the agent

#### Setup using visual studio:
1. Install visual studio professional (https://visualstudio.microsoft.com/it/vs/professional/)
2. Install cmake, clang and make 
3. Copy from the folder lib/win_visual_studio the communication library to the folder lib/
4. Compile the agent:
   - In Clion:
     1. Open the folder project (in CLion: create Debug and Release profile);
     2. Select the correct compiler Visual Studio (https://www.jetbrains.com/help/clion/how-to-create-toolchain-in-clion.html)
     3. Compile the agent
5. Run the agent

#### Setup using MinGW
1. Install MSYS2 https://www.msys2.org/
2. Install cmake, clang and make (using the shell of Msys2)
```bash
pacman -S mingw-w64-x86_64-cmake 
pacman -S mingw-w64-x86_64-clang 
pacman -S mingw-w64-x86_64-make
```
3. Copy from the folder lib/win_mingw the communication library to the folder lib/
4. Compile the agent:
   - In Clion:
      1. Open the folder project and create Debug and Release profile
      2. Select the correct compiler WLS (follow the guide https://www.jetbrains.com/help/clion/how-to-create-toolchain-in-clion.html)
      3. Compile the agent

   - In a windows terminal (cmd):
     1. check which compiler is used:
     ```bash
     cmake --help
     ```
     2. run cmake:
     ```bash
     cmake -G "MSYS Makefiles"
     ```
     3. Build the agent:
     ```bash
     cmake --build .
     ```
5. Run the agent

### Linux
1. Open a Linux shell and run:
```bash
sudo apt-get update
sudo apt-get install cmake
sudo apt-get install clang
```
3. Copy from the folder lib/linux or lib/linux_fPIC the communication library to the folder lib/
4. Compile the agent:
   - In Clion:
      1. Open the folder project and create Debug and Release profile
      2. Select the correct compiler WLS (follow the guide https://www.jetbrains.com/help/clion/how-to-create-toolchain-in-clion.html)
      3. Compile the agent
   - In the linux terminal:
      1. Compile the agent using the following command:
      ```bash
      cmake .
      make
      ```
5. Run the agent

### MacOs
1. Open a shell and run:
```bash
brew update
brew install cmake
brew install clang
```
3. Copy from the folder lib/macos or lib/macos_x86 the communication library to the folder lib/
4. Compile the agent:
   - In Clion:
      1. Open the folder project and create Debug and Release profile
      2. Select the correct compiler WLS (follow the guide https://www.jetbrains.com/help/clion/how-to-create-toolchain-in-clion.html)
      3. Compile the agent
   - In the linux terminal:
      1. Compile the agent using the following command:
      ```bash
      cmake .
      make
      ```
5. Run the agent