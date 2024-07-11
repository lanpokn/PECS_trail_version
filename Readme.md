# Physical-based Event Camera Simulator

This is the official implementation of Physical-based Event Camera Simulator（ **PECS**）.If you need more information, please refer to the paper with the same title.

![Image](.\output\backbone.jpg)



### (1) Use_Main

This code has been tested with Python 3.9.13 and blender 3.6. The usage instructions for PECS are as follow

- install <a href="https://github.com/mmp/pbrt-v4">pbrt</a>, and Replace the files in PBRT with the corresponding files in this project(in pbrt folder)

(The complete modified version of pbrt's github repository address will be released after the paper is accepted)

- Install blender and it's pbrt add-on

- generate .pbrt file with specified lenses file,  for example: rotate.pbrt

  you can refer to pbrt-v3 book for more information about Realistic Camera lens

- run pbrt and get the exr file, for example rotate.exr

  ```
  pbrt -v rotate.pbrt
  ```

- run sensor module to get the final result. The sensor/src part is partly from ICNS. In addtition, some functions need **metavision** to run. If you don't want to use metavision, you can comment them out.

- you can refer to sensor/src/example_EXR_to_events.py to get some examples.

### (2) Use_Airsim

you can also use Airsim to generate event data real-time. The usage instructions are as follows:

- Install <a href="https://github.com/microsoft/AirSim">AirSim</a>

- use  [eventcamera_sim](airsim\eventcamera_sim)  to replace the folder with the same name in Airsim

- generate library for eventcamera_sim

```
python sensor/utils/setup_display.py build
```

and move the output library to the [eventcamera_sim](airsim\eventcamera_sim)  folder. Then you can use PECS in Airsim.

### (3) Others

If you want to use more features, please refer to the various functions in **example_EXR_to_events.py**, which include metric metrics, 3D display, video display and other functions.

you can also try ESIM in Esim.py, and download <a href="https://github.com/SensorsINI/v2e">V2E</a>, PECS provides an interface to read the output data of these simulators for subsequent processing.

### (4) Result

The figure below shows the actual data obtained from the rotational version captured in a professional darkroom, compared with the simulated data output from various simulators,demonstrating the **accuracy** of PECS.

![Image](.\output\R_all.jpg)

Similarly, the figure below shows various data obtained from capturing a translated checkerboard pattern in a professional darkroom.

![Image](.\output\T_all.jpg)

The figure below shows the output when various complex large 3D scenes are used as PECS input, demonstrating the **versatility** of PECS.

![Image](.\output\Various_scene.jpg)

### Acknowledgment

-  Part of our code refers to <a href="https://github.com/mmp/pbrt-v4">pbrt</a>  and  <a href="https://github.com/neuromorphicsystems/IEBCS">ICNS</a>.
-  We use blender and UE(airsim) as tools


### Updates

26/11/2023: Initial release