from numba.np.ufunc import parallel
import numpy as np
from types import SimpleNamespace
from numba import njit, prange, set_num_threads
import dsi
from extern.event_buffer import EventBuffer
from extern.event_display import EventDisplay
from extern.dvs_sensor import init_bgn_hist_cpp

EVENT_TYPE = np.dtype(
    [("timestamp", "f8"), ("x", "u2"), ("y", "u2"), ("polarity", "b")], align=True
)

TOL = 0.5
MINIMUM_CONTRAST_THRESHOLD = 0.01

CONFIG = SimpleNamespace(
    **{
        "contrast_thresholds": (0.01, 0.01),
        "sigma_contrast_thresholds": (0.0, 0.0),
        "refractory_period_ns": 1000,
        "max_events_per_frame": 200000,
    }
)

@njit(parallel=True)
def esim(
    x_end,
    current_image,
    previous_image,
    delta_time,
    crossings,
    last_time,
    output_events,
    spikes,
    refractory_period_ns,
    max_events_per_frame,
    n_pix_row,
):
    count = 0
    max_spikes = int(delta_time / (refractory_period_ns * 1e-3))
    for x in prange(x_end):
        itdt = np.log(current_image[x])
        it = np.log(previous_image[x])
        deltaL = itdt - it

        if np.abs(deltaL) < TOL:
            continue

        pol = np.sign(deltaL)

        cross_update = pol * TOL
        crossings[x] = np.log(crossings[x]) + cross_update

        lb = crossings[x] - it
        ub = crossings[x] - itdt

        pos_check = lb > 0 and (pol == 1) and ub < 0
        neg_check = lb < 0 and (pol == -1) and ub > 0

        spike_nums = (itdt - crossings[x]) / TOL
        cross_check = pos_check + neg_check
        spike_nums = np.abs(int(spike_nums * cross_check))

        crossings[x] = itdt - cross_update
        if spike_nums > 0:
            spikes[x] = pol

        spike_nums = max_spikes if spike_nums > max_spikes else spike_nums

        current_time = last_time
        for i in range(spike_nums):
            output_events[count].x = x % n_pix_row
            output_events[count].y = x // n_pix_row
            output_events[count].timestamp = np.round(current_time * 1e-6, 6)
            output_events[count].polarity = 1 if pol > 0 else -1

            count += 1
            current_time += (delta_time) / spike_nums

            if count == max_events_per_frame:
                return count

    return count


class EventSimulator:
    def __init__(self, W, H, first_image=None, first_time=None, config=CONFIG):
        self.H = H
        self.W = W
        self.config = config
        self.last_image = None
        if first_image is not None:
            assert first_time is not None
            self.init(first_image, first_time)

        self.npix = H * W

    def init(self, first_image, first_time):
        print("Initialized event camera simulator with sensor size:", first_image.shape)

        self.resolution = first_image.shape  # The resolution of the image

        # We ignore the 2D nature of the problem as it is not relevant here
        # It makes multi-core processing more straightforward
        first_image = first_image.reshape(-1)

        # Allocations
        self.last_image = first_image.copy()
        self.current_image = first_image.copy()

        self.last_time = first_time

        self.output_events = np.zeros(
            (self.config.max_events_per_frame), dtype=EVENT_TYPE
        )
        self.event_count = 0
        self.spikes = np.zeros((self.npix))

    def image_callback(self, new_image, new_time):
        if self.last_image is None:
            self.init(new_image, new_time)
            return None, None

        assert new_time > 0
        assert new_image.shape == self.resolution
        new_image = new_image.reshape(-1)  # Free operation

        np.copyto(self.current_image, new_image)

        delta_time = new_time - self.last_time

        config = self.config
        self.output_events = np.zeros(
            (self.config.max_events_per_frame), dtype=EVENT_TYPE
        )
        self.spikes = np.zeros((self.npix))

        self.crossings = self.last_image.copy()
        self.event_count = esim(
            self.current_image.size,
            self.current_image,
            self.last_image,
            delta_time,
            self.crossings,
            self.last_time,
            self.output_events,
            self.spikes,
            config.refractory_period_ns,
            config.max_events_per_frame,
            self.W,
        )

        np.copyto(self.last_image, self.current_image)
        self.last_time = new_time

        result = self.output_events[: self.event_count]
        result.sort(order=["timestamp"], axis=0)

        return self.spikes, result

class EventSimulatorNew:
    def __init__(self, W, H, first_image=None, first_time=None, config=CONFIG):
        self.H = H
        self.W = W
        self.config = config
        self.last_image = None
        if first_image is not None:
            assert first_time is not None
            self.init(first_image, first_time)

        self.npix = H * W

    def init(self, first_image, first_time):
        print("Initialized event camera simulator with sensor size:", first_image.shape)

        self.resolution = first_image.shape  # The resolution of the image

        # We ignore the 2D nature of the problem as it is not relevant here
        # It makes multi-core processing more straightforward
        # first_image = first_image.reshape(-1)

        lat = 100
        jit = 10
        ref = 100
        tau = 300
        th = 0.3
        th_noise = 0.01
        dsi.initSimu(int(first_image.shape[0]), int(first_image.shape[1]))
        dsi.initLatency(lat, jit, ref, tau)
        dsi.initContrast(th, th, th_noise)
        dsi.initImg(first_image)
        init_bgn_hist_cpp("C:/Users/admin/Documents/2023/PBES/my_pbrt/IEBCS-main/data/noise_neg_161lux.npy", "C:/Users/admin/Documents/2023/PBES/my_pbrt/IEBCS-main/data/noise_neg_161lux.npy")
        ev_full = EventBuffer(1)
        # self.ed = EventDisplay("Events", int(first_image.shape[0]+1), int(first_image.shape[1]+1), 500000)
        # Allocations
        self.last_image = first_image.copy()
        self.current_image = first_image.copy()

        self.last_time = first_time

        self.output_events = np.zeros(
            (self.config.max_events_per_frame), dtype=EVENT_TYPE
        )
        self.event_count = 0
        self.spikes = np.zeros((self.npix))

    def image_callback(self, new_image, new_time):
        if self.last_image is None:
            self.init(new_image, new_time)
            return None, None

        assert new_time > 0
        assert new_image.shape == self.resolution

        np.copyto(self.current_image, new_image)

        delta_time = new_time - self.last_time

        config = self.config
        self.output_events = np.zeros((self.config.max_events_per_frame), dtype=EVENT_TYPE)
        self.spikes = np.zeros((self.npix))

        self.crossings = self.last_image.copy()

        buf = dsi.updateImg(self.current_image, int(delta_time))
        ev = EventBuffer(1)
        ev.add_array(np.array(buf["ts"], dtype=np.uint64),
                    np.array(buf["x"], dtype=np.uint16),
                    np.array(buf["y"], dtype=np.uint16),
                    np.array(buf["p"], dtype=np.uint64),
                    100000)

        range = min(ev.get_ts().shape[0], config.max_events_per_frame)

        output_events_x = self.output_events["x"][:range]
        output_events_y = self.output_events["y"][:range]
        output_events_timestamp = self.output_events["timestamp"][:range]
        output_events_polarity = self.output_events["polarity"][:range]

        np.copyto(output_events_x, ev.get_x()[:range])
        np.copyto(output_events_y, ev.get_y()[:range])
        np.copyto(output_events_timestamp, ev.get_ts()[:range])
        np.copyto(output_events_polarity, np.where(ev.get_p()[:range] > 0, 1, -1))

        self.spikes = np.zeros((self.npix))
        spikes_y = ev.get_y()[:range]
        spikes_x = ev.get_x()[:range]
        spikes_p = ev.get_p()[:range]
        self.spikes[spikes_y * self.W + spikes_x] = np.where(spikes_p > 0, 1, -1)

        np.copyto(self.last_image, self.current_image)
        self.last_time = new_time

        result = self.output_events[:range]
        result.sort(order=["timestamp"], axis=0)

        return self.spikes, result