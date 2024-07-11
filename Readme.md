# Physical-based Event Camera Simulator

This is the official implementation of Physical-based Event Camera Simulator（ **PECS**）.This is a trial version used to validate the algorithm. Since we plan to replace PBRT in the future, this version has actually been deprecated, and PECS will be migrated to a better major platform. If you need more information, please refer to the paper with the same title.

![Image](.\output\backbone.jpg)



### (1) Use_Main
This version of the PBRT renderer has actually been deprecated by us due to its slow speed, limited functionality, and the frequent, headache-inducing complex compilation issues it causes. However, to demonstrate how the data in our original paper was produced, we have included this modified version of PBRT. Nonetheless, we strongly recommend that you use a renderer you find more comfortable and try to add lens simulation capabilities (in fact, we are working on this ourselves).

Or you can ignore rendering engine and only use (multi-spectral) camera video as input to test quantum efficiency

This code has been tested with Python 3.9.13 and blender 3.6. The usage instructions for PECS are as follow

- install <a href="https://github.com/mmp/pbrt-v4">pbrt</a>, and Replace the files in PBRT with the corresponding files in this project(in pbrt_utils folder)
(it's OK to just use original pbrt-v4, ignoring my pbrt_utils folder. Howerer it may hard to generate long video)

- Install blender and it's pbrt add-on

- generate .pbrt file with specified lenses file,  for example: rotate.pbrt

  you can refer to pbrt-v3 book for more information about Realistic Camera lens

- run pbrt and get the exr file, for example rotate.exr

  ```
  pbrt -v rotate.pbrt
  ```

- run sensor module to get the final result. The sensor/src part is partly from ICNS because our algorithm is unrelated to the electronic circuitry except for the photoelectric effect.Thus, we use ICNS to achieve electronic circuitry simulation. In addtition, some functions need **metavision** to run because our camera is EVK4. If you don't want to use metavision, you can comment them out.

- you can refer to sensor/src/example_EXR_to_events.py to get the example of how my paper's result is generated. please note that you need to generate photocurrent based on your own camera's quantum efficency. we have implemented it in film.cpp(line 937) to get an additional channel when rendering, and sensor/utils when using RGB video.

### (2) Use_Airsim

you can also use Airsim to generate event data real-time. The usage instructions are as follows:

- Install <a href="https://github.com/microsoft/AirSim">AirSim</a>

- use  [eventcamera_sim](airsim\eventcamera_sim)  to replace the folder with the same name in Airsim

- generate library for eventcamera_sim

```
python sensor/utils/setup_display.py build
```

and move the output library to the [eventcamera_sim](airsim\eventcamera_sim)  folder. Then you can use PECS in Airsim.
(you may need some adds-on to support lens simulation)

### (3) Others

PECS's algorithm contribution is firstly proposed the effect of lens simulatio and quantum efficency. This means you can combine arbitory sensor and rendering engine, and plus above algorithms to get a better simulaton result in a specific camera.

We are developing a more industrialized and user-friendly official version to replace this rough and difficult-to-use experimental version of the project.

### (4) Result

Details in the paper.

### Acknowledgment

-  Part of our code refers to <a href="https://github.com/mmp/pbrt-v4">pbrt</a>  and  <a href="https://github.com/neuromorphicsystems/IEBCS">ICNS</a>.
-  We use blender and UE(airsim) as tools


### Updates
