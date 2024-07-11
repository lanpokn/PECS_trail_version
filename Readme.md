# Physical-based Event Camera Simulator

This is the official implementation of Physical-based Event Camera Simulator（ **PECS**）.This is a trial version used to validate the algorithm. Since PBRT has actually been deprecated by us due to its slow speed, limited functionality, and the frequent, headache-inducing complex compilation issues it causes, this version only has UE version. If you need more information, please refer to the paper with the same title.



### (1) Ext introduction

- The sensor/src part is partly from ICNS because our algorithm is unrelated to the electronic circuitry except for the photoelectric effect.Thus, we use ICNS to achieve electronic circuitry simulation. In addtition, some functions need **metavision** to run because our camera is EVK4. If you don't want to use metavision, you can comment them out.

-please note that you need to generate photocurrent based on your own camera's quantum efficency.Sensor/utils is an example of quantum efficency implementation.

### (2) Use_Airsim

we edit Airsim to generate event data real-time in UE. The usage instructions are as follows:

- Install <a href="https://github.com/microsoft/AirSim">AirSim</a>

- use  [eventcamera_sim](airsim\eventcamera_sim)  to replace the folder with the same name in Airsim

- generate library for eventcamera_sim

```
python sensor/utils/setup_display.py build
```

and move the output library to the [eventcamera_sim](airsim\eventcamera_sim)  folder. 
Then you need to generate a dsi lib and move it to the [eventcamera_sim](airsim\eventcamera_sim)  folder, which you can refer to <a href="https://github.com/neuromorphicsystems/IEBCS">ICNS</a>. Finaly, you can use PECS in Airsim. 

### (3) Other suggestions

PECS's algorithm contribution is firstly proposed the effect of lens simulation and quantum efficency. This means you can combine arbitory sensor with rendering engine you like, and using above algorithms to get a better simulaton result in a specific camera.

### (4) Result

Details in the paper.(tested in windows 11)

### Acknowledgment

-  Part of our code refers to  <a href="https://github.com/neuromorphicsystems/IEBCS">ICNS</a>
-  We use blender and UE(airsim) as tools

### Updates
We may implement the algorithmic concepts of PECS into a larger project in the future, and there will be further updates at that time.
