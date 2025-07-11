# [emachinery](https://pypi.org/project/emachinery/)

*<u>A package for analysis of electric machinery.</u>*

# Instructions to use emy-c

## [A]. Install

### Developer 
To run the source code, git clone the repo and create a virtual environment via conda:
```shell
conda create --name emy python=3.10 streamlit=1.32.2 numpy pandas control matplotlib numba openpyxl psutil pyyaml rich sympy
conda config --add channels conda-forge
conda config --set channel_priority strict
conda install pygmo
```
The package further requires the following free softwares to compile c source codes:
- gcc (comes with [Minimalist GNU for Windows](https://sourceforge.net/projects/mingw/)---See this [awesome page](https://www3.ntu.edu.sg/home/ehchua/programming/howto/Cygwin_HowTo.html) for info)
- gmake.exe (I use the one from [TI's CCS](https://www.ti.com/tool/download/CCSTUDIO), it is located at somewhere like: `D:\ti\ccs930\ccs\utils\bin\gmake.exe`. Anyway, I decided to just copy-paste gmake.exe to `/emachinery/acmsimc/c/gmake.exe`. So no need to install CCS anymore.)
  - It also works with CMake with some minor modification.

### fast launch tip streamlit for inteacting with emy-c
```shell
<win+R>
wt -w _quake
cd ~
notepad .\emy-go.bat
<enter>
cd c:\_codes\emy-c\emachinery   && conda activate emy   && python .\main.py
<ctrl+s>
em <tab> <enter>
```

### General Users
Alternatively, the package can be installed via pip.
```shell
pip install emachinery
emy
```
Command `emy` is an entry point for starting the main program of this package, which is equivalent to `python emachinery/main.py`

## [B]. Functions (outdated)

### 1. Machine Quantities Calculations & Conversion

### 2. PI Regulator Tuner based on Texas Instruments' InstaSPIN

### 3. Run C-based Numerical Integration Simulation

### 4. Sweep Frequency Analysis

### 5. Run Python- and Numba-based Realtime Numerical Integration Simulation

### 6. FEA based Machine Design and Multi-Objective Optimization (develop)

### 7. PC User GUI for Serial Communication to DSP (develop)


## [C]. Simulation Tips

1. Field oriented control is an asymptotic input-output linearizing (IOL) control. This means the IOL is achieved only when flux modulus is regulated to its reference. So before motor starts, we must wait for the motor to build up its magnetic air gap field or else the starting transient would be disturbed. Refer to the figure below.![1](https://github.com/horychen/emachinery/blob/main/gallery/readme-pic-flux-to-build.png?raw=true)
2. The inductance of the motor matters. For a small "DC" servo PM motor, the inductance is relatively designed to be low because the DC source is usually only 24 V or 48 V. However, an induction motor usually has higher inductance, which limits the bandwidth of the current loop, I think. For example, this is what happens to d-axis current regulation when I set desired velocity loop bandwidth from 50 Hz up to 100 Hz. Refer to the figure below.![2](https://github.com/horychen/emachinery/blob/main/gallery/readme-pic-comparison-d-axis-current-regulation-per-inductance.png?raw=true)



## [D]. Known Issues (outdated)

### GNU Make failed to create process
After adding installing CCS and MinGW, you might need to restart the PC.

```
gcc -o main pmsm_comm.c im_controller.c ... -I. -L.
process_begin: CreateProcess(NULL, gcc -o main pmsm_comm.c im_controller.c... -I. -L., ...) failed.
make (e=2): The system cannot find the file specified.
makefile:11: recipe for target 'main' failed
gmake: *** [main] Error 2
```

See this [question](https://stackoverflow.com/questions/3848357/createprocess-no-such-file-or-directory)

## [E] Other Information

_This package is published to PYPI following https://www.youtube.com/watch?v=Qs91v2Tofys_

----

