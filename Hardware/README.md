# <center>3D Models</center>

<p align="center"><img src="../Images/3D_Model.png" width="500"></p>


## Custom PCB

The PCB can be populated with either an AS5600 or an MT6701 encoder. The MCU can automatically figure out which one is connected. However at the moment there are only non blocking drivers for the AS5600.

There are two versions of the PCB. One for the GD32F130 microcontroller, which is also used by Feetech servos. The other one uses a CH32V003 instead, which is cheaper. However no firmware has been written yet for the CH32V003 and the PCB has not been tested.


### Programming Dongles

The GD32F130 is programmed with a ST-LINK V2 dongle: https://www.aliexpress.us/item/3256805941217455.html / https://www.aliexpress.us/item/3256805308308820.html

The CH32V003  is programmed with a WCH-linkE dongle: https://www.aliexpress.us/item/3256804084637506.html



### Solder Stencil

[Cutting solder paste stencils from Mylar YouTube video](https://www.youtube.com/watch?v=mw0mskVCvis)
[make_stencil.py GitHub](https://github.com/bminch/Eclectronics)

- Open KicCAD PCB => Plot 
- Select layers: F. Paste and Edge Cuts
- Plot Format: SVG
- python make_stencil.py gd32_encoder

