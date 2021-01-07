# emachinery

*<u>A package for analysis of electric machinery.</u>*

## A. Functions

### 1. Machine Quantities Calculations & Conversion

### 2. PI Regulator Tuner based on Texas Instruments' InstaSPIN

### 3. Run C-based Numerical Integration Simulation

### 4. Sweep Frequency Analysis

### 5. FEA based Machine Design and Multi-Objective Optimization (develop)

### 6. PC User GUI for Serial Communication to DSP (develop)


## B. Installation and Requirements

The package can be installed via pip.
```shell
pip install emachinery
```

The package requires the following free softwares to work as expected:
- qtconsole (can be installed via pip, see also https://github.com/jupyter/qtconsole)
- gmake.exe (can be installed with TI's CCS, it is located at *D:\ti\ccs930\ccs\utils\bin\gmake.exe*)
- pylab (numpy, matplotlib, ...)
- pyqt5
- [Optional] Qt Designer (can be download from https://build-system.fman.io/qt-designer-download) 



## C. Simulation Tips

1. Field oriented control is an asymptotic input-output linearizing (IOL) control. This means the IOL is achieved only when flux modulus is regulated to its reference. So before motor start, we must wait for the motor to build up its magnetic air gap field or else the start transient would be disturbed. Refer to the figure below.![1](https://github.com/horychen/emachinery/blob/main/gallery/readme-pic-flux-to-build.png?raw=true)
2. The inductance of the motor matters. For a small "DC" servo PM motor, the inductance is relatively designed to be low because the DC source is usually only 24 V or 48 V. However, an induction motor usually has higher inductance, which limits the bandwidth of the current loop, I think. For example, this is what happens to d-axis current regulation when I set desired velocity loop bandwidth from 50 Hz up to 100 Hz. Refer to the figure below.![2](https://github.com/horychen/emachinery/blob/main/gallery/readme-pic-comparison-d-axis-current-regulation-per-inductance.png?raw=true)



## Known Issues
After I upgrade my jupyterlab to version 3, qtconsole (version 4.6.0) starts to not work well. Below is some error message.
```
Traceback (most recent call last):
  File "D:\DrH\Codes\emachineryTestPYPI\emachinery\gui\main.py", line 947, in <module>
    main()
  ...
  ...
  ...
  File "D:\DrH\Codes\emachineryTestPYPI\emachinery\gui\consolewidget.py", line 31, in __init__
    self.kernel_client = kernel_client = self._kernel_manager.client()
  File "D:\Users\horyc\Anaconda3\lib\site-packages\qtconsole\base_frontend_mixin.py", line 63, in kernel_client
    if kernel_client.channels_running:
  File "D:\Users\horyc\Anaconda3\lib\site-packages\jupyter_client\client.py", line 141, in channels_running
    self.control_channel.is_alive())
  File "D:\Users\horyc\Anaconda3\lib\site-packages\jupyter_client\client.py", line 200, in control_channel
    socket, self.session, self.ioloop
TypeError: ChannelABC() takes no arguments
```
I found a related discussion here: https://github.com/jupyter/jupyter_client/issues/523
Reading the webpage, I fix this problem by first uninstalling ipykernel and install it back to a higher version.
```
pip uninstall ipykernel
Found existing installation: ipykernel 5.1.4
Uninstalling ipykernel-5.1.4:

pip install ipykernel
Collecting ipykernel
  Downloading ipykernel-5.4.2-py3-none-any.whl (119 kB)
```

----
_This package is published to PYPI following https://www.youtube.com/watch?v=Qs91v2Tofys_

----

