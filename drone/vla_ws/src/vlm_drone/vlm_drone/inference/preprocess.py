import numpy as np
import cv2

def preprocess_frame(frame_bgr: np.ndarray, resize_hw=None) -> np.ndarray:
    """
    frame_bgr: OpenCV BGR image (H,W,3), uint8 preferred.
    resize_hw: (H, W) or None
    """
    if frame_bgr is None:
        raise ValueError("frame_bgr is None")

    if frame_bgr.dtype != np.uint8:
        frame_bgr = frame_bgr.astype(np.uint8)

    if resize_hw is not None:
        h, w = resize_hw
        frame_bgr = cv2.resize(frame_bgr, (w, h), interpolation=cv2.INTER_LINEAR)

    return frame_bgr
