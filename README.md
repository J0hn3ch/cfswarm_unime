
Crazyflie UniMe Project
=======================

Testbed and scripts to interact with Crazyflie
---------------------------------------------



# 1. Getting Started
## 1.1 Requirements
* usbipd
* python

## 1.2 Windows & WSL2 - Attach Crazyradio 
1. Attach the dongle Crazyradio 2.0 to USB port
2. Use the command `usbipd list` to see the list of USB device to manage
3. Use the command `usbipd bind --busid 1-2`
4. Use the command `usbipd attach --wsl --busid 1-2` to make the dongle reachable from WSL 


## 1.3 Edge nodes
### Edge Node 1
1. SSH `ssh -L 8081:localhost:8081 universome@192.168.137.240 -p 2222`
2. Move to folder : `cd Desktop\crazyflie-unime-edge-1`
3. Jupyter Server : `jupyter notebook --no-browser --port=8081`

_Install dependencies_
* Install Jupyter : `python -m pip install --no-cache-dir --verbose notebook`
* Install Crazyflie Python Library : `python -m pip install --no-cache-dir --verbose cflib`
* Install IPython kernel for Jupyter : `python -m pip install --no-cache-dir --verbose -U ipykernel`

_Errors:_
* If you stuck on scipy installation, check for `sudo apt install libopenblas-dev`

### Edge Node 2
1. SSH : `ssh universome@192.168.137.4 -p 22`
2. Move to folder : `cd crazyflie-unime-edge-2`

_Install dependencies_
* Install Jupyter : `python -m pip install --no-cache-dir --verbose notebook`
* Install Crazyflie Python Library : `python -m pip install --no-cache-dir --verbose cflib`
* Install IPython kernel for Jupyter : `python -m pip install --no-cache-dir --verbose -U ipykernel`
* Install Matplotlib : `python -m pip install --no-cache-dir --verbose -U matplotlib`


**Matplotlib issues**
1. [AttributeError: 'TimedAnimation' object has no attribute '_framedata'](https://github.com/matplotlib/matplotlib/issues/30831)
2. [Logging data with Matplotlib Animation issues](https://github.com/bitcraze/crazyflie-lib-python/issues/584)

**NOT WORKING - VisPy**
_Dependencies:_
* jupyter_rfb - https://github.com/vispy/jupyter_rfb - `python -m pip install jupyter_rfb`
* vispy : `python -m pip install --upgrade vispy`
* pytest : `python -m pip install "pytest<8.0" --no-binary pytest`
* AAA - flake8 : `python -m pip install --upgrade flake8`
* Lark : `python -m pip install lark --upgrade`
* Backend for OpenGL context - pyglet : `python -m pip install --upgrade pyglet`
* 

**Cairo**
1. https://www.cairographics.org/pycairo/