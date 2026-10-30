#!/usr/bin/env python3
"""Export the existing ARD-MAV checkpoints; Python is only needed for this offline step."""

import argparse
import hashlib
import importlib.util
import json
import shutil
import tempfile
from pathlib import Path

import onnx
import torch
import ultralytics
from ultralytics import YOLO


def export_detector(source, destination):
    """Stage a checkpoint so Ultralytics never writes beside the training weights."""
    with tempfile.TemporaryDirectory(prefix="ard_mav_detector_") as temporary:
        checkpoint = Path(temporary) / source.name
        shutil.copy2(source, checkpoint)
        model = YOLO(checkpoint)
        if (
            model.task != "detect"
            or model.names != {0: "Drone"}
            or not model.model.model[-1].end2end
        ):
            raise ValueError("Expected a single-class Drone end-to-end YOLO26 detector")
        metadata = {
            "source": str(source),
            "source_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
            "checkpoint_epoch": model.ckpt.get("epoch"),
            "checkpoint_date": model.ckpt.get("date"),
            "ultralytics": ultralytics.__version__,
            "torch": torch.__version__,
        }
        exported = model.export(
            format="onnx",
            imgsz=640,
            dynamic=True,
            opset=17,
            simplify=False,
            nms=False,
            max_det=100,
            device="cpu",
            half=False,
        )
        onnx.checker.check_model(onnx.load(exported))
        shutil.copy2(exported, destination)
    metadata["onnx_sha256"] = hashlib.sha256(destination.read_bytes()).hexdigest()
    return metadata


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--tracking",
        type=Path,
        default=package.parents[1] / "ultralytics/ARD-MAV/tracking",
    )
    parser.add_argument("--output", type=Path, default=package / "models")
    parser.add_argument(
        "--local-weights", type=Path, help="Export only this new local checkpoint"
    )
    parser.add_argument(
        "--local-name", help="New .onnx filename; existing models are never overwritten"
    )
    args = parser.parse_args()
    if bool(args.local_weights) != bool(args.local_name):
        parser.error("--local-weights and --local-name must be used together")
    tracking, output = args.tracking.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.local_weights:
        filename = Path(args.local_name)
        if filename.name != args.local_name or filename.suffix != ".onnx":
            parser.error("--local-name must be a filename ending in .onnx")
        destination = output / filename
        if destination.exists():
            raise FileExistsError(
                f"Model already exists; choose a new --local-name: {destination}"
            )
        manifest_path = output / "manifest.json"
        manifest = (
            json.loads(manifest_path.read_text())
            if manifest_path.exists()
            else {"models": {}}
        )
        manifest["models"][filename.name] = export_detector(
            args.local_weights.expanduser().resolve(), destination
        )
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"Exported new local model to {destination}; existing models preserved")
        return
    spec = importlib.util.spec_from_file_location(
        "ard_mav_motion", tracking / "scripts/motion.py"
    )
    motion = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(motion)
    manifest_path = output / "manifest.json"
    manifest = (
        json.loads(manifest_path.read_text())
        if manifest_path.exists()
        else {"models": {}}
    )
    manifest.update(torch=torch.__version__, ultralytics=ultralytics.__version__)
    for stem in ("global_best", "local_best", "Net_best"):
        source = (
            tracking / "weights" / (stem + (".pth" if stem == "Net_best" else ".pt"))
        )
        destination = output / f"{stem}.onnx"
        if stem != "Net_best":
            export_detector(source, destination)
        else:
            model = motion.AppearanceClassifier().eval()
            model.load_state_dict(
                torch.load(source, map_location="cpu", weights_only=True),
                strict=True,
            )
            torch.onnx.export(
                model,
                torch.zeros(1, 3, 32, 32),
                str(destination),
                input_names=["images"],
                output_names=["logits"],
                opset_version=17,
                dynamic_axes={"images": {0: "batch"}, "logits": {0: "batch"}},
                dynamo=False,
            )
        onnx.checker.check_model(onnx.load(destination))
        manifest["models"][destination.name] = {
            "source": source.name,
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "onnx_sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
        }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Exported three models to {output}")


if __name__ == "__main__":
    main()
