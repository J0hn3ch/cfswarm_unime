
Crazyflie UniMe Project
=======================

Testbed and scripts to interact with Crazyflie
---------------------------------------------



# 1. Getting Started
# 1.1 Requirements
* usbipd
* python

# 1.2 Windows & WSL2 - Attach Crazyradio 
1. Attach the dongle Crazyradio 2.0 to USB port
2. Use the command `usbipd list` to see the list of USB device to manage
3. Use the command `usbipd bind --wsl --busid 1-2`
4. Use the command `usbipd attach --wsl --busid 1-2` to make the dongle reachable from WSL 

**Matplotlib issues**
1. [AttributeError: 'TimedAnimation' object has no attribute '_framedata'](https://github.com/matplotlib/matplotlib/issues/30831)
2. [Logging data with Matplotlib Animation issues](https://github.com/bitcraze/crazyflie-lib-python/issues/584)

**Cairo**
1. https://www.cairographics.org/pycairo/

20 SLIDE
PROBLEMA 
COME LO AFFRONTIAMO
SPECIFICHE DEL SISTEMA
FOCUS SUGLI ELEMENTI PRINCIPALI 
PROTOCOLLO CNP
ORCHESTRAZIONE DA AGENTI SUL CAMPO
INFRASTRUTTURA FOG 
AGENTIC (FUTURE WORK)

IN TESI stiamo lavorando in questi termini
RISULTATI dei dati 