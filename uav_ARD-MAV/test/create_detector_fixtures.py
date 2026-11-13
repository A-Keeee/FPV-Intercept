"""Regenerate tiny ONNX fixtures for deterministic search/reacquisition tests.

Requires onnx only when regenerating; the C++ tests load the committed models.
The global fixture always misses. The local fixture reports a fixed box with
confidence equal to the input center, so a uniform gray target can be switched
on/off without motion features or dependence on trained model predictions.
The scalable fixture uses image brightness to vary box size for ROI switching.
The fixed-square fixture rejects inputs other than 640x640 to guard letterboxing.
"""

from pathlib import Path

import onnx
from onnx import TensorProto, helper


def save(name, nodes, input_shape=None):
    graph = helper.make_graph(
        nodes,
        name,
        [helper.make_tensor_value_info("image", TensorProto.FLOAT, input_shape or [1, 3, "h", "w"])],
        [helper.make_tensor_value_info("detections", TensorProto.FLOAT, [1, 1, 6])],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    destination = Path(__file__).parent / "fixtures" / name
    destination.parent.mkdir(exist_ok=True)
    onnx.save(model, destination)


def constant(name, dims, values, dtype=TensorProto.FLOAT):
    return helper.make_node(
        "Constant", [], [name], value=helper.make_tensor(name, dtype, dims, values)
    )


def brightness_nodes():
    # Sample inside the image, so the 114-valued letterbox border cannot turn a
    # blank frame into a detection or change the requested target brightness.
    return [
        helper.make_node("Shape", ["image"], ["image_shape"]),
        constant("spatial_axes", [2], [2, 3], TensorProto.INT64),
        helper.make_node("Gather", ["image_shape", "spatial_axes"], ["spatial_shape"], axis=0),
        constant("two", [2], [2, 2], TensorProto.INT64),
        helper.make_node("Div", ["spatial_shape", "two"], ["center_yx"]),
        constant("batch_channel", [2], [0, 0], TensorProto.INT64),
        helper.make_node("Concat", ["batch_channel", "center_yx"], ["center_indices"], axis=0),
        helper.make_node("GatherND", ["image", "center_indices"], ["brightness"]),
    ]


save("global_miss.onnx", [constant("detections", [1, 1, 6], [320, 160, 480, 320, 0, 0])])
save(
    "local_brightness.onnx",
    brightness_nodes() + [
        constant("axes", [3], [0, 1, 2], TensorProto.INT64),
        helper.make_node("Unsqueeze", ["brightness", "axes"], ["score"]),
        constant("box", [1, 1, 4], [320, 240, 480, 400]),
        constant("class", [1, 1, 1], [0]),
        helper.make_node("Concat", ["box", "score", "class"], ["detections"], axis=2),
    ],
)

save(
    "scalable_brightness.onnx",
    brightness_nodes() + [
        constant("half_size_scale", [], [300]),
        helper.make_node("Mul", ["brightness", "half_size_scale"], ["half_size"]),
        constant("center", [2], [320, 320]),
        helper.make_node("Sub", ["center", "half_size"], ["low"]),
        helper.make_node("Add", ["center", "half_size"], ["high"]),
        helper.make_node("Concat", ["low", "high"], ["coordinates"], axis=0),
        constant("box_axes", [2], [0, 1], TensorProto.INT64),
        helper.make_node("Unsqueeze", ["coordinates", "box_axes"], ["box"]),
        constant("score_scale", [], [5]),
        helper.make_node("Mul", ["brightness", "score_scale"], ["unclipped_score"]),
        constant("score_max", [], [0.95]),
        helper.make_node("Min", ["unclipped_score", "score_max"], ["confidence"]),
        constant("score_axes", [3], [0, 1, 2], TensorProto.INT64),
        helper.make_node("Unsqueeze", ["confidence", "score_axes"], ["score"]),
        constant("class", [1, 1, 1], [0]),
        helper.make_node("Concat", ["box", "score", "class"], ["detections"], axis=2),
    ],
)

save(
    "fixed_square_brightness.onnx",
    brightness_nodes() + [
        constant("axes", [3], [0, 1, 2], TensorProto.INT64),
        helper.make_node("Unsqueeze", ["brightness", "axes"], ["score"]),
        constant("box", [1, 1, 4], [160, 240, 480, 400]),
        constant("class", [1, 1, 1], [0]),
        helper.make_node("Concat", ["box", "score", "class"], ["detections"], axis=2),
    ],
    input_shape=[1, 3, 640, 640],
)
