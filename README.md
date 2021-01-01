# emachinery

*<u>A package for analysis of electric machinery.</u>*

## A. Functions

### 1. Machine Quantities Calculations & Conversion

### 2. PI Regulator Tuner based on Texas Instruments' InstaSPIN

### 3. Run C-based Numerical Integration Simulation

### 4. Sweep Frequency Analysis

### 5. FEA based Machine Design and Multi-Objective Optimization (develop)

### 6. PC User GUI for Serial Communication to DSP (develop)

## Requirements

The package emachinery requires the following free softwaresto work as expected:
- qtconsole (can be installed via pip, see also https://github.com/jupyter/qtconsole)
- gmake.exe (can be installed with TI's CCS, it is located at *D:\ti\ccs930\ccs\utils\bin\gmake.exe*)
- pylab (numpy, matplotlib, ...)
- pyqt5
- [Optional] Qt Designer (can be download from https://build-system.fman.io/qt-designer-download) 



## Simulation Tips

1. Field oriented control is a asymptotic input-output linearizing (IOL) control. This means the IOL is achieved only when flux modulus is regulated to its reference. So before motor start, we must wait for the motor to build up its magnetic air gap field or else the start transient would be disturbed. Refer to the figure below.![1](https://github.com/horychen/emachinery/blob/main/gallery/readme-pic-flux-to-build.png?raw=true)
2. The inductance of the motor matters. For a small "DC" servo PM motor, the inductance is relatively designed to be low because the DC source is usually only 24 V or 48 V. However, an induction motor usually has higher inductance, which limits the bandwidth of the current loop, I think. For example, this is what happens to d-axis current regulation when I set desired velocity loop bandwidth from 50 Hz up to 100 Hz. Refer to the figure below.![2](https://github.com/horychen/emachinery/blob/main/gallery/readme-pic-comparison-d-axis-current-regulation-per-inductance.png?raw=true)




----
_This package is published following https://www.youtube.com/watch?v=Qs91v2Tofys_

----

