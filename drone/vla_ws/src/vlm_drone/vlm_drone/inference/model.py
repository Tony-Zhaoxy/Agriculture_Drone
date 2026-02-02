import time
import cv2
import requests
import numpy as np

class HTTPVLAModel:
    """
    Calls local Flask server:
      POST /predict_action
      files: image (jpeg)
      form: instruction
    Returns:
      {
        "velocities": {"x":..., "y":..., "z":...},
        "delta_yaw": ...
      }
    """

    def __init__(self, server_url: str, timeout_s: float = 5.0, jpeg_quality: int = 80):
        self.server_url = server_url
        self.timeout_s = timeout_s
        self.jpeg_quality = int(jpeg_quality)

    def predict(self, frame_bgr: np.ndarray, instruction: str) -> dict:
        if frame_bgr is None:
            raise ValueError("frame_bgr is None")

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        ok, buf = cv2.imencode(".jpg", frame_bgr, encode_params)
        if not ok:
            raise RuntimeError("cv2.imencode(.jpg) failed")

        t0 = time.perf_counter()
        resp = requests.post(
            self.server_url,
            files={"image": ("frame.jpg", buf.tobytes(), "image/jpeg")},
            data={"instruction": instruction},
            timeout=self.timeout_s,
        )
        dt = time.perf_counter() - t0

        if resp.status_code != 200:
            raise RuntimeError(f"Server error {resp.status_code}: {resp.text}")

        result = resp.json()
        result["_rtt_sec"] = float(dt)
        return result

def parse_action(result: dict):
    """
    Convert server result -> [vx, vy, vz, dyaw]
    """
    vel = result.get("velocities", {})
    vx = vel.get("x", 0.0)
    vy = vel.get("y", 0.0)
    vz = vel.get("z", 0.0)
    dyaw = result.get("delta_yaw", 0.0)
    return [float(vx), float(vy), float(vz), float(dyaw)]
