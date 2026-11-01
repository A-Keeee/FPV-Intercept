#!/usr/bin/env python3
"""Run the unmodified Ultralytics tracker in an isolated process, using a local binary stream."""

import argparse
import os
import struct
import signal
import sys
from types import SimpleNamespace


def read_exact(size):
    chunks = bytearray()
    while len(chunks) < size:
        part = sys.stdin.buffer.read(size - len(chunks))
        if not part:
            raise EOFError
        chunks.extend(part)
    return chunks


def main():
    # Reserve stdout for the protocol, including during third-party imports.
    output = sys.stdout.buffer
    sys.stdout = sys.stderr
    os.environ["YOLO_AUTOINSTALL"] = "false"
    import numpy as np
    import torch
    import ultralytics
    import yaml
    from ultralytics.engine.results import Boxes
    from ultralytics.trackers.bot_sort import BOTSORT
    from ultralytics.trackers.byte_tracker import BYTETracker

    signal.signal(
        signal.SIGINT, signal.SIG_IGN
    )  # The owning C++ process handles terminal shutdown.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    with open(args.config) as stream:
        settings = yaml.safe_load(stream)
    if settings["tracker_type"] not in {"botsort", "bytetrack"}:
        raise ValueError("Supported native trackers: botsort, bytetrack")
    if not 0 <= settings["track_low_thresh"] < settings["track_high_thresh"] <= 1:
        raise ValueError("Require 0 <= track_low_thresh < track_high_thresh <= 1")
    if not settings["track_high_thresh"] <= settings["new_track_thresh"] <= 1:
        raise ValueError("new_track_thresh must be >= track_high_thresh and <= 1")
    if settings["track_buffer"] < 0 or not 0 < settings["match_thresh"] <= 1:
        raise ValueError("Invalid track_buffer/match_thresh")
    if settings.get("with_reid", False):
        raise ValueError(
            "This deployment uses motion association only; with_reid must be false"
        )
    torch.set_num_threads(1)
    tracker_class = BOTSORT if settings["tracker_type"] == "botsort" else BYTETracker
    tracker = tracker_class(SimpleNamespace(**settings), frame_rate=30)
    print(
        f"Ultralytics {ultralytics.__version__}: native {tracker_class.__name__}, internal Kalman enabled"
    )
    output.write(struct.pack("=I", 0x59544B31))
    output.flush()
    while True:
        try:
            command, height, width, count, image_bytes = struct.unpack(
                "=5I", read_exact(20)
            )
        except EOFError:
            return
        if command == 1:
            tracker.reset()
            tracks = np.empty((0, 8), np.float32)
        elif command == 0:
            if (
                not 0 < image_bytes <= 256 * 1024 * 1024
                or image_bytes != height * width * 3
                or count > 100
            ):
                raise ValueError("Invalid frame/detection payload")
            detections = (
                np.frombuffer(read_exact(count * 6 * 4), dtype="=f4")
                .reshape(count, 6)
                .copy()
            )
            image = (
                np.frombuffer(read_exact(image_bytes), dtype=np.uint8)
                .reshape(height, width, 3)
                .copy()
            )
            # Same update entry point used by model.track's on_predict_postprocess_end callback.
            # Both global and local observations arrive in ORIGINAL image coordinates.
            tracks = np.asarray(
                tracker.update(Boxes(detections, (height, width)), image), np.float32
            ).reshape(-1, 8)
        else:
            raise ValueError(f"Unknown protocol command: {command}")
        output.write(struct.pack("=I", len(tracks)))
        output.write(tracks.astype("=f4", copy=False).tobytes())
        output.flush()


if __name__ == "__main__":
    main()
