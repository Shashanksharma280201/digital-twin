"""Left rectified images from a ZED ``.svo2`` recording (needs the ZED SDK's ``pyzed``).

The SVO holds every grabbed frame (up to 30 Hz) without the CPU JPEG step the bag's image
topic goes through, so it is the preferred colour source when a session has one.
Stamps are the SDK image timestamps, on the same clock as the wrapper's ROS stamps.
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import numpy as np


class Svo:
    def __init__(self, path: str | Path, scale: float = 1.0):
        import pyzed.sl as sl

        self.sl, self.scale = sl, scale
        self.cam = sl.Camera()
        init = sl.InitParameters()
        init.set_from_svo_file(str(path))
        init.svo_real_time_mode = False
        init.depth_mode = sl.DEPTH_MODE.NONE
        err = self.cam.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"cannot open {path}: {err}")
        conf = self.cam.get_camera_information().camera_configuration
        left = conf.calibration_parameters.left_cam
        self.width, self.height = round(conf.resolution.width * scale), round(conf.resolution.height * scale)
        self.K = np.array([[left.fx * scale, 0, left.cx * scale], [0, left.fy * scale, left.cy * scale], [0, 0, 1]])
        self.frames = self.cam.get_svo_number_of_frames()

    def __enter__(self) -> "Svo":
        return self

    def __exit__(self, *exc) -> None:
        self.cam.close()

    def images(self, lazy: bool = False) -> Iterator[tuple[float, np.ndarray]]:
        """(stamp seconds, BGR uint8) for every frame, in order.

        ``lazy``: yield a callable instead of the image; only frames whose callable is invoked
        (before the next frame) are copied out and resized.
        """
        sl = self.sl
        mat = sl.Mat()
        size = sl.Resolution(self.width, self.height)

        def fetch():
            self.cam.retrieve_image(mat, sl.VIEW.LEFT, sl.MEM.CPU, size)  # SDK resizes on the GPU
            return mat.get_data()[:, :, :3].copy()

        while True:
            err = self.cam.grab()
            if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                return
            if err != sl.ERROR_CODE.SUCCESS:
                continue
            stamp = self.cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds() * 1e-9
            yield stamp, (fetch if lazy else fetch())
