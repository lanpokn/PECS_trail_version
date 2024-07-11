# Haiqian Han
"""
TODO

"""
import cv2
import sys
sys.path.append("../../src")
from event_buffer import EventBuffer
from dvs_sensor import init_bgn_hist_cpp, DvsSensor
from event_display import EventDisplay
import dsi
import numpy as np
from ExrRead import read_exr_channel,calculate_intensity_from_spetral

from event_file_io import EventsData
import numpy as np
from scipy.spatial import KDTree
import math
import time
import cv2
from event_loss import *
import open3d as o3d
from skimage.transform import resize, rescale
import random

def Rotate_360_high(C = 100):
    start_time = time.time()
    lat = 10
    jit = 10
    ref = 100
    tau = 300
    th = 0.3
    th_noise = 0.01

    #rotate_360_high, 100-10=100-3-0.3-0.01
    width = 1280
    height = 720
    dsi.initSimu(int(height), int(width))
    dsi.initLatency(lat, jit, ref, tau)
    dsi.initContrast(th, th, th_noise)
    init_bgn_hist_cpp("D:/2023/computional imaging/my_pbrt/IEBCS-main/data/noise_pos_161lux.npy", "D:/2023/computional imaging/my_pbrt/IEBCS-main/data/noise_pos_161lux.npy")
    isInit = False
    dt = 2857
    ev_full = EventBuffer(1)
    ed = EventDisplay("Events", width, height, dt)
    time_event = 0
    out = cv2.VideoWriter('rotate_360_highlight_{}_{}_{}_{}_{}_{}_nonoise.avi'.format(lat, jit, ref, tau, th, th_noise),
        cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20.0, (int(width), int(height)))
    i = 0
    while i<20:
        filename = "D:/2023/computional imaging/my_pbrt/build/Rotate_360_pbrt001"+str(i+20)+".exr"
        im = read_exr_channel(filename,"intensity",C)
        cv2.imshow("t", im)
        cv2.waitKey(1)
        im = im*750
        out.write(im)
        i=i+1
        if im is None:
            break
        if not isInit:
            dsi.initImg(im)
            isInit = True
        else:
            buf = dsi.updateImg(im, dt)
            ev = EventBuffer(1)
            ev.add_array(np.array(buf["ts"], dtype=np.uint64),
                            np.array(buf["x"], dtype=np.uint16),
                            np.array(buf["y"], dtype=np.uint16),
                            np.array(buf["p"], dtype=np.uint64),
                            1000000)
            ed.update(ev, dt)
            ev_full.increase_ev(ev)
            time_event += dt
            if time_event > 0.1e7:
                break
    out.release()
    ev_full.write('D:/2023/computional imaging/my_pbrt/output/Rotate_360_high/R_360_H_PBES.dat'.format(lat, jit, ref, tau, th, th_noise))
    end_time = time.time()
    total_time = end_time - start_time
    print("Total time of PBES", total_time)

def View_3D(point_cloud):
    # Create a visualizer object
    vis = o3d.visualization.Visualizer()
    # Add the point cloud to the visualizer
    vis.create_window()
    vis.add_geometry(point_cloud)
    # Get the view control object
    view_control = vis.get_view_control()

    # Set the viewpoint from the left
    view_control.set_lookat([1, 0, 0])  # Look towards positive X-axis
    view_control.set_front([-1, 0, 0])  # Set negative X-axis as the front direction

    # Run the visualizer
    vis.run()
    # Destroy the visualizer window
    vis.destroy_window()
def Compare_Real_and_PBES(Realpath, simPath,time_intervel = 100000,C=100,N=31):
    """use it to test functions
    """    
    events_data = EventsData()
    events_data_IEBCS = EventsData()
    #make sure the video is long enough, or it can't disolay normally
    events_data.read_real_events(Realpath, time_intervel)
    events_data_IEBCS.read_IEBCS_events(simPath, time_intervel)
    #3D output is the best way to calibrate
    ev_data0 = events_data.events[0]
    ev_data1 = events_data_IEBCS.events[0]



    # chamfer
    start_time = time.time()
    loss_same = chamfer_distance_loss(ev_data0, ev_data0)
    print(loss_same)
    loss_different = chamfer_distance_loss(ev_data0, ev_data1)
    print(loss_different)
    end_time = time.time()
    total_time = end_time - start_time
    print("Total time of chamfer method", total_time)
    # gausian
    start_time = time.time()
    loss_same = gaussian_distance_loss(ev_data0, ev_data0)
    print(loss_same)
    loss_different = gaussian_distance_loss(ev_data0, ev_data1)
    print(loss_different)
    
    end_time = time.time()
    total_time = end_time - start_time
    print("Total time of gausian method", total_time)
Rotate_360_high()
Compare_Real_and_PBES("D:/2023/computional imaging/my_pbrt/output/Rotate_360_high/High_360_120deg.hdf5","D:/2023/computional imaging/my_pbrt/output/Rotate_360_high/R_360_H_PBES.dat",100000)
