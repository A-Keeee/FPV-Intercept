"""Regenerate tiny ONNX fixtures for deterministic search/reacquisition tests.

Requires onnx only when regenerating; the C++ tests load the committed models.
The global fixture always misses. The local fixture reports a fixed box with
confidence equal to the input maximum, so a uniform gray target can be switched
on/off without motion features or dependence on trained model predictions.
"""

from pathlib import Path

import onnx
from onnx import TensorProto, helper


def save(name, nodes):
    graph = helper.make_graph(
        nodes,
        name,
        [helper.make_tensor_value_info("image", TensorProto.FLOAT, [1, 3, "h", "w"])],
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


save("global_miss.onnx", [constant("detections", [1, 1, 6], [320, 160, 480, 320, 0, 0])])
save(
    "local_brightness.onnx",
    [
        helper.make_node("ReduceMax", ["image"], ["brightness"], keepdims=0),
        constant("axes", [3], [0, 1, 2], TensorProto.INT64),
        helper.make_node("Unsqueeze", ["brightness", "axes"], ["score"]),
        constant("box", [1, 1, 4], [320, 160, 480, 320]),
        constant("class", [1, 1, 1], [0]),
        helper.make_node("Concat", ["box", "score", "class"], ["detections"], axis=2),
    ],
)
