import argparse

import onnx
import numpy as np
from onnx import numpy_helper, helper


def transform(path):
    model = onnx.load(path)
    for value in list(model.graph.input) + list(model.graph.output):
        dim = value.type.tensor_type.shape.dim[0]
        dim.ClearField("dim_value")
        dim.dim_param = "batch"
    consumers = {}
    for node in model.graph.node:
        for item in node.input:
            consumers.setdefault(item, []).append(node)
    constants = {}
    for node in model.graph.node:
        if node.op_type == "Constant":
            for attribute in node.attribute:
                if attribute.type == onnx.AttributeProto.TENSOR:
                    constants[node.output[0]] = (
                        attribute,
                        numpy_helper.to_array(attribute.t),
                    )
    reshape_shapes = {node.input[1] for node in model.graph.node if node.op_type == "Reshape"}
    expand_data = {node.input[0] for node in model.graph.node if node.op_type == "Expand"}
    batch_flatten_shapes = {
        node.input[1]
        for node in model.graph.node
        if node.op_type == "Reshape"
        and node.name
        in {
            "/Reshape_10",
            "/Reshape_32",
            "/Reshape_34",
            "/Reshape_45",
            "/Reshape_48",
            "/Reshape_68",
            "/Reshape_118",
            "/Reshape_126",
            "/Reshape_128",
            "/Reshape_150",
        }
    }
    attention_output_shapes = {
        node.input[1]
        for node in model.graph.node
        if node.op_type == "Reshape" and "self_attn/Reshape_9" in node.name
    }
    attention_restore_shapes = {
        node.input[1]
        for node in model.graph.node
        if node.op_type == "Reshape" and "self_attn/Reshape_10" in node.name
    }
    attention_mask_shapes = {
        node.input[1]
        for node in model.graph.node
        if node.op_type == "Reshape" and "self_attn/Reshape_7" in node.name
    }
    attention_root_output_shapes = {
        node.input[1]
        for node in model.graph.node
        if node.op_type == "Reshape" and "self_attn/Reshape_12" in node.name
    }
    attention_root_restore_shapes = {
        node.input[1]
        for node in model.graph.node
        if node.op_type == "Reshape" and "self_attn/Reshape_13" in node.name
    }
    attention_qkv_shapes = {
        node.input[1]
        for node in model.graph.node
        if node.op_type == "Reshape"
        and "self_attn/Reshape_" in node.name
        and node.name.rsplit("_", 1)[-1] in {"3", "4", "5"}
    }
    for name, (attribute, value) in constants.items():
        if (
            (name not in reshape_shapes and name not in batch_flatten_shapes)
            or value.ndim != 1
            or not value.size
        ):
            continue
        rewritten = value.copy()
        if name in attention_qkv_shapes and rewritten.size == 3:
            # MultiheadAttention receives [sequence, batch, embedding]. Keep
            # the sequence axis fixed and fold batch into the head axis.
            rewritten[1] = -1
        elif name in attention_root_restore_shapes:
            rewritten[1] = -1
        elif name in attention_root_output_shapes:
            rewritten[0] = -1
        elif name in attention_mask_shapes and rewritten[0] == 16:
            rewritten[0] = -1
        elif name in attention_restore_shapes:
            rewritten[1] = -1
        elif name in attention_output_shapes:
            rewritten[0] = -1
        elif (
            rewritten.size == 3
            and rewritten[-1] in {32, 64}
            and 16 in rewritten
            and -1 not in rewritten
        ):
            rewritten[np.where(rewritten == 16)[0][0]] = -1
        elif name in batch_flatten_shapes:
            rewritten[0] = -1
        elif rewritten[0] == 1:
            rewritten[0] = 0
        else:
            continue
        attribute.t.CopyFrom(numpy_helper.from_array(rewritten))
    prefix = [
        helper.make_node("Shape", ["context_mujoco_qpos"], ["spark_ctx_shape"]),
        helper.make_node(
            "Gather", ["spark_ctx_shape", "spark_zero"], ["spark_batch_scalar"], axis=0
        ),
        helper.make_node("Unsqueeze", ["spark_batch_scalar", "spark_axes0"], ["spark_batch_vec"]),
    ]
    model.graph.initializer.extend(
        [
            numpy_helper.from_array(np.array(0, np.int64), name="spark_zero"),
            numpy_helper.from_array(np.array([0], np.int64), name="spark_axes0"),
            numpy_helper.from_array(np.array([1], np.int64), name="spark_axes1"),
            numpy_helper.from_array(np.array(4, np.int64), name="spark_four"),
            numpy_helper.from_array(np.array([29], np.int64), name="spark_29"),
            numpy_helper.from_array(np.array([1], np.int64), name="spark_one_vec"),
            numpy_helper.from_array(np.array(1, np.int64), name="spark_one_scalar"),
            numpy_helper.from_array(np.array([1, 1], np.int64), name="spark_one_one"),
            numpy_helper.from_array(np.array([1, 2], np.int64), name="spark_axes12"),
            numpy_helper.from_array(np.array([1, 2, 3], np.int64), name="spark_axes123"),
            numpy_helper.from_array(np.array([1, 2, 3, 4], np.int64), name="spark_axes1234"),
            numpy_helper.from_array(np.array([2], np.int64), name="spark_axes2"),
            numpy_helper.from_array(np.array([4, 4], np.int64), name="spark_four_four"),
            numpy_helper.from_array(np.array([1, 512], np.int64), name="spark_one_512"),
            numpy_helper.from_array(np.array([60, 1], np.int64), name="spark_60_one"),
            numpy_helper.from_array(np.array([64], np.int64), name="spark_64"),
            numpy_helper.from_array(np.array([128], np.int64), name="spark_128"),
            numpy_helper.from_array(np.array([3], np.int64), name="spark_three_vec"),
            numpy_helper.from_array(np.array([4], np.int64), name="spark_four_vec"),
            numpy_helper.from_array(np.array([36], np.int64), name="spark_36_vec"),
            numpy_helper.from_array(np.array([2], np.int64), name="spark_two_vec"),
            numpy_helper.from_array(np.array([1, 9], np.int64), name="spark_one_nine"),
            numpy_helper.from_array(np.array([9], np.int64), name="spark_nine"),
            numpy_helper.from_array(
                constants["onnx::Expand_1393"][1].copy(), name="spark_expand1393_base"
            ),
            numpy_helper.from_array(
                constants["/Constant_842_output_0"][1].copy(), name="spark_scatter842_base"
            ),
            numpy_helper.from_array(
                constants["/Constant_706_output_0"][1].copy(), name="spark_scatter706_base"
            ),
            numpy_helper.from_array(
                constants["/Constant_1281_output_0"][1].copy(), name="spark_root_quat_base"
            ),
            numpy_helper.from_array(
                constants["/Constant_1308_output_0"][1].copy(), name="spark_root_position_base"
            ),
            numpy_helper.from_array(
                constants["/Constant_1265_output_0"][1].copy(), name="spark_root_heading_base"
            ),
            numpy_helper.from_array(
                constants["/Constant_1498_output_0"][1].copy(), name="spark_qpos_base"
            ),
        ]
    )
    replacements = {
        "/Constant_252_output_0": "spark_batch4_29",
        "/Constant_246_output_0": "spark_batch4_1_1",
        "onnx::Expand_1393": "spark_expand_1393",
        "/Constant_842_output_0": "spark_scatter842_batched",
        "/Constant_706_output_0": "spark_scatter706_batched",
        "/Constant_1281_output_0": "spark_root_quat_batched",
        "/Constant_1308_output_0": "spark_root_position_batched",
        "/Constant_1265_output_0": "spark_root_heading_batched",
        "/Constant_1498_output_0": "spark_qpos_batched",
        "onnx::Concat_12760": "spark_root_start_token",
        "/mmm_net/Constant_72_output_0": "spark_batch_60_1",
    }
    preamble = [
        helper.make_node("Mul", ["spark_batch_scalar", "spark_four"], ["spark_batch4_scalar"]),
        helper.make_node("Unsqueeze", ["spark_batch4_scalar", "spark_axes0"], ["spark_batch4_vec"]),
        helper.make_node("Concat", ["spark_batch4_vec", "spark_29"], ["spark_batch4_29"], axis=0),
        helper.make_node(
            "Concat", ["spark_batch4_vec", "spark_one_vec"], ["spark_batch4_1"], axis=0
        ),
        helper.make_node(
            "Concat", ["spark_batch4_vec", "spark_one_one"], ["spark_batch4_1_1"], axis=0
        ),
        helper.make_node("Concat", ["spark_batch_vec", "spark_one_vec"], ["spark_batch_1"], axis=0),
        helper.make_node(
            "Range",
            ["spark_zero", "spark_batch4_scalar", "spark_one_scalar"],
            ["spark_batch4_range"],
        ),
        helper.make_node(
            "Range", ["spark_zero", "spark_batch_scalar", "spark_one_scalar"], ["spark_batch_range"]
        ),
        helper.make_node(
            "Unsqueeze", ["spark_batch4_range", "spark_axes1"], ["spark_batch4_range_column"]
        ),
        helper.make_node(
            "Expand", ["spark_batch4_range_column", "spark_batch4_29"], ["spark_expand_1393"]
        ),
        helper.make_node(
            "Concat", ["spark_batch_vec", "spark_four_four"], ["spark_batch_4_4"], axis=0
        ),
        helper.make_node(
            "Expand", ["spark_scatter842_base", "spark_batch_4_4"], ["spark_scatter842_batched"]
        ),
        helper.make_node(
            "Expand", ["spark_scatter706_base", "spark_batch_4_4"], ["spark_scatter706_batched"]
        ),
        helper.make_node(
            "Concat", ["spark_batch_vec", "spark_one_512"], ["spark_batch_1_512"], axis=0
        ),
        helper.make_node(
            "Expand", ["onnx::Concat_12760", "spark_batch_1_512"], ["spark_root_start_token"]
        ),
        helper.make_node(
            "Concat", ["spark_batch_vec", "spark_60_one"], ["spark_batch_60_1"], axis=0
        ),
        helper.make_node(
            "Concat", ["spark_batch_vec", "spark_two_vec"], ["spark_batch_two"], axis=0
        ),
        helper.make_node(
            "Concat", ["spark_batch_vec", "spark_one_one"], ["spark_batch_1_1"], axis=0
        ),
        helper.make_node(
            "Concat",
            ["spark_batch_vec", "spark_64", "spark_three_vec"],
            ["spark_batch_64_3"],
            axis=0,
        ),
        helper.make_node(
            "Concat", ["spark_batch_vec", "spark_64"], ["spark_batch_64_root"], axis=0
        ),
        helper.make_node(
            "Concat", ["spark_batch_vec", "spark_64", "spark_36_vec"], ["spark_batch_64_36"], axis=0
        ),
        helper.make_node(
            "Concat",
            ["spark_batch_vec", "spark_64", "spark_four_vec"],
            ["spark_batch_64_4"],
            axis=0,
        ),
        helper.make_node(
            "Expand", ["spark_root_quat_base", "spark_batch_64_4"], ["spark_root_quat_batched"]
        ),
        helper.make_node(
            "Expand",
            ["spark_root_position_base", "spark_batch_64_3"],
            ["spark_root_position_batched"],
        ),
        helper.make_node(
            "Expand",
            ["spark_root_heading_base", "spark_batch_64_root"],
            ["spark_root_heading_batched"],
        ),
        helper.make_node(
            "Expand", ["spark_qpos_base", "spark_batch_64_36"], ["spark_qpos_batched"]
        ),
    ]
    counter = 0
    for name, (_, value) in constants.items():
        if (
            value.dtype == np.int64
            and value.ndim == 1
            and value.size >= 2
            and value[0] == 1
            and name not in expand_data
            and any(
                node.op_type in {"Reshape", "Expand", "ConstantOfShape", "Equal"}
                for node in consumers.get(name, [])
            )
        ):
            rest = f"spark_dynamic_rest_{counter}"
            output = f"spark_dynamic_shape_{counter}"
            preamble.extend(
                [
                    helper.make_node(
                        "Constant", [], [rest], value=numpy_helper.from_array(value[1:].copy())
                    ),
                    helper.make_node("Concat", ["spark_batch_vec", rest], [output], axis=0),
                ]
            )
            replacements[name] = output
            counter += 1
    rewritten_nodes = []
    expanded = 0
    for node in model.graph.node:
        for index, item in enumerate(list(node.input)):
            if item in replacements:
                node.input[index] = replacements[item]
        if node.name in {
            "/Slice_70",
            "/Slice_75",
            "/Slice_76",
            "/Slice_82",
            "/Slice_86",
            "/Slice_102",
            "/Slice_106",
        }:
            node.input[2] = "spark_batch_vec"
        axis = next((attribute.i for attribute in node.attribute if attribute.name == "axis"), 0)
        if node.op_type == "Concat" and axis != 0:
            for index, item in enumerate(list(node.input)):
                if item not in constants:
                    continue
                value = constants[item][1]
                if value.ndim < 2 or value.shape[0] != 1:
                    continue
                rest = f"spark_expand_rest_{expanded}"
                shape = f"spark_expand_shape_{expanded}"
                output = f"spark_expand_output_{expanded}"
                rewritten_nodes.extend(
                    [
                        helper.make_node(
                            "Constant",
                            [],
                            [rest],
                            value=numpy_helper.from_array(np.array(value.shape[1:], np.int64)),
                        ),
                        helper.make_node("Concat", ["spark_batch_vec", rest], [shape], axis=0),
                        helper.make_node("Expand", [item, shape], [output]),
                    ]
                )
                node.input[index] = output
                expanded += 1
        if node.name == "/Sub_7":
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze", [node.input[1], "spark_axes1"], ["spark_specific_target_origin"]
                )
            )
            node.input[1] = "spark_specific_target_origin"
        if node.name in {"/Mul_198", "/Mul_199"}:
            mask_name = (
                "spark_moving_heading_mask"
                if node.name == "/Mul_198"
                else "spark_static_heading_mask"
            )
            heading_name = mask_name + "_values"
            rewritten_nodes.append(
                helper.make_node(
                    "Reshape",
                    [node.input[0], "spark_batch_1"],
                    [mask_name],
                    name=mask_name + "_reshape",
                )
            )
            node.input[0] = mask_name
            rewritten_nodes.append(
                helper.make_node(
                    "Reshape",
                    [node.input[1], "spark_batch_two"],
                    [heading_name],
                    name=heading_name + "_reshape",
                )
            )
            node.input[1] = heading_name
        if node.name in {"/Mul_196", "/Mul_197"}:
            mask_name = node.name.replace("/", "spark_") + "_mask"
            value_name = node.name.replace("/", "spark_") + "_values"
            rewritten_nodes.append(
                helper.make_node(
                    "Reshape",
                    [node.input[0], "spark_batch_1"],
                    [mask_name],
                    name=mask_name + "_reshape",
                )
            )
            node.input[0] = mask_name
            rewritten_nodes.append(
                helper.make_node(
                    "Reshape",
                    [node.input[1], "spark_batch_1"],
                    [value_name],
                    name=value_name + "_reshape",
                )
            )
            node.input[1] = value_name
        if node.name in {"/Mul_207", "/Mul_208"}:
            mask_name = node.name.replace("/", "spark_") + "_mask"
            rewritten_nodes.append(
                helper.make_node(
                    "Reshape",
                    [node.input[0], "spark_batch_1_1"],
                    [mask_name],
                    name=mask_name + "_reshape",
                )
            )
            node.input[0] = mask_name
        if node.name == "/Mul_217":
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze", [node.input[0], "spark_axes12"], ["spark_planner_time"]
                )
            )
            node.input[0] = "spark_planner_time"
        if node.name == "/Add_96":
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze", [node.input[0], "spark_axes12"], ["spark_planner_heading"]
                )
            )
            node.input[0] = "spark_planner_heading"
        if node.name == "/Add_97":
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze", [node.input[1], "spark_axes12"], ["spark_planner_heading_offset"]
                )
            )
            node.input[1] = "spark_planner_heading_offset"
        if node.name == "/Add_102":
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze", [node.input[1], "spark_axes1"], ["spark_planner_random_seed"]
                )
            )
            node.input[1] = "spark_planner_random_seed"
        if node.name == "/Sub_65":
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze", [node.input[1], "spark_axes1"], ["spark_planner_token_count"]
                )
            )
            node.input[1] = "spark_planner_token_count"
        if node.name == "/Add_104":
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze", [node.input[1], "spark_axes1"], ["spark_planner_token_offset"]
                )
            )
            node.input[1] = "spark_planner_token_offset"
        if node.name in {"/Mul_228", "/Mul_229"}:
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze",
                    [node.input[1], "spark_axes1"],
                    [f"spark_planner_token_scale_{node.name.rsplit('_', 1)[-1]}"],
                )
            )
            node.input[1] = f"spark_planner_token_scale_{node.name.rsplit('_', 1)[-1]}"
        if node.name == "/Mul_230":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Squeeze", [node.input[0], "spark_axes0"], ["spark_planner_gather_squeezed"]
                    ),
                    helper.make_node(
                        "Transpose",
                        ["spark_planner_gather_squeezed"],
                        ["spark_planner_gather_batched"],
                        perm=[1, 0, 2, 3],
                    ),
                ]
            )
            node.input[0] = "spark_planner_gather_batched"
        if node.name == "/Mul_231":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Squeeze",
                        [node.input[0], "spark_axes0"],
                        ["spark_planner_gather_pose_squeezed"],
                    ),
                    helper.make_node(
                        "Transpose",
                        ["spark_planner_gather_pose_squeezed"],
                        ["spark_planner_gather_pose_batched"],
                        perm=[1, 0, 2, 3, 4],
                    ),
                ]
            )
            node.input[0] = "spark_planner_gather_pose_batched"
        if node.name == "/Mul_232":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Squeeze",
                        [node.input[0], "spark_axes0"],
                        ["spark_planner_gather_rotation_squeezed"],
                    ),
                    helper.make_node(
                        "Transpose",
                        ["spark_planner_gather_rotation_squeezed"],
                        ["spark_planner_gather_rotation_batched"],
                        perm=[1, 0, 2, 3, 4, 5],
                    ),
                ]
            )
            node.input[0] = "spark_planner_gather_rotation_batched"
        if node.name == "/Mul_233":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Squeeze",
                        [node.input[0], "spark_axes0"],
                        ["spark_planner_gather_count_squeezed"],
                    ),
                    helper.make_node(
                        "Transpose",
                        ["spark_planner_gather_count_squeezed"],
                        ["spark_planner_gather_count_batched"],
                        perm=[1, 0, 2],
                    ),
                ]
            )
            node.input[0] = "spark_planner_gather_count_batched"
        if node.name in {"/Reshape_63", "/Reshape_67"}:
            suffix = node.name.rsplit("_", 1)[-1]
            rewritten_nodes.append(
                helper.make_node(
                    "Expand", [node.input[0], node.input[1]], [f"spark_planner_valid_mask_{suffix}"]
                )
            )
            node.input[0] = f"spark_planner_valid_mask_{suffix}"
        if node.name in {"/Reshape_124", "/Reshape_141"}:
            suffix = node.name.rsplit("_", 1)[-1]
            rewritten_nodes.append(
                helper.make_node(
                    "Expand",
                    [node.input[0], node.input[1]],
                    [f"spark_planner_root_output_batched_{suffix}"],
                )
            )
            node.input[0] = f"spark_planner_root_output_batched_{suffix}"
        if node.name == "/Reshape_81":
            node.input[1] = "spark_batch_vec"
        if node.name == "/Reshape_88":
            rewritten_nodes.append(
                helper.make_node(
                    "Concat", ["spark_batch_vec", "spark_three_vec"], ["spark_batch_3"], axis=0
                )
            )
            node.input[1] = "spark_batch_3"
        if node.name == "/Reshape_134":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Unsqueeze",
                        [node.input[0], "spark_axes2"],
                        ["spark_planner_rotations_unsqueezed"],
                    ),
                    helper.make_node(
                        "Expand",
                        ["spark_planner_rotations_unsqueezed", node.input[1]],
                        ["spark_planner_rotations_batched"],
                    ),
                ]
            )
            node.input[0] = "spark_planner_rotations_batched"
        if node.name == "/Reshape_117":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Unsqueeze",
                        [node.input[0], "spark_axes2"],
                        ["spark_planner_pose_mask_unsqueezed"],
                    ),
                    helper.make_node(
                        "Expand",
                        ["spark_planner_pose_mask_unsqueezed", node.input[1]],
                        ["spark_planner_pose_mask_batched"],
                    ),
                ]
            )
            node.input[0] = "spark_planner_pose_mask_batched"
        if node.name == "/Reshape_143":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Concat", ["spark_batch_vec", "spark_one_nine"], ["spark_batch_1_9"], axis=0
                    ),
                    helper.make_node(
                        "Expand",
                        [node.input[0], "spark_batch_1_9"],
                        ["spark_planner_initial_rotation_batched"],
                    ),
                ]
            )
            node.input[0] = "spark_planner_initial_rotation_batched"
        if node.name == "/Reshape_145":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Concat",
                        ["spark_batch_vec", "spark_64", "spark_nine"],
                        ["spark_batch_64_9"],
                        axis=0,
                    ),
                    helper.make_node(
                        "Expand",
                        [node.input[0], "spark_batch_64_9"],
                        ["spark_planner_rotation_sequence_batched"],
                    ),
                ]
            )
            node.input[0] = "spark_planner_rotation_sequence_batched"
        if node.name == "/Tile_1":
            rewritten_nodes.append(
                helper.make_node(
                    "Reshape", [node.input[0], "spark_batch_1"], ["spark_planner_frame_indexes"]
                )
            )
            node.input[0] = "spark_planner_frame_indexes"
        if node.name == "/Less_15":
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze", [node.input[1], "spark_axes1"], ["spark_planner_frame_count"]
                )
            )
            node.input[1] = "spark_planner_frame_count"
        if node.name == "/ScatterElements_1":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Concat", ["spark_batch_vec", "spark_128"], ["spark_batch_128"], axis=0
                    ),
                    helper.make_node(
                        "Expand",
                        [node.input[0], "spark_batch_128"],
                        ["spark_pose_token_candidates"],
                    ),
                ]
            )
            node.input[0] = "spark_pose_token_candidates"
        if node.name == "/Concat_111":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Concat",
                        ["spark_batch_vec", "spark_64", "spark_one_vec"],
                        ["spark_batch_64_1"],
                        axis=0,
                    ),
                    helper.make_node(
                        "Expand",
                        [node.input[0], "spark_batch_64_3"],
                        ["spark_decoded_root_position"],
                    ),
                    helper.make_node(
                        "Expand", [node.input[1], "spark_batch_64_1"], ["spark_decoded_root_cos"]
                    ),
                    helper.make_node(
                        "Expand", [node.input[2], "spark_batch_64_1"], ["spark_decoded_root_sin"]
                    ),
                ]
            )
            node.input[0] = "spark_decoded_root_position"
            node.input[1] = "spark_decoded_root_cos"
            node.input[2] = "spark_decoded_root_sin"
        if node.name == "/Add_173":
            rewritten_nodes.append(
                helper.make_node(
                    "Unsqueeze", [node.input[1], "spark_axes1"], ["spark_planner_root_offset"]
                )
            )
            node.input[1] = "spark_planner_root_offset"
        if node.name in {"/Mul_292", "/Mul_297"}:
            # torch.multinomial flattens [B, classes] and indexes each row with
            # a batch-stride offset.  The batch-one export hard-codes that row
            # index to zero.
            node.input[0] = "spark_batch_range"
        if node.name in {
            "/Unsqueeze_115",
            "/Unsqueeze_118",
            "/Unsqueeze_136",
            "/Unsqueeze_146",
            "/Unsqueeze_152",
            "/Unsqueeze_155",
            "/Unsqueeze_158",
            "/Unsqueeze_165",
            "/Unsqueeze_171",
            "/Unsqueeze_174",
            "/Unsqueeze_177",
            "/Unsqueeze_180",
            "/Unsqueeze_185",
            "/Unsqueeze_203",
            "/Unsqueeze_206",
            "/Unsqueeze_209",
            "/Unsqueeze_212",
            "/Unsqueeze_201",
            "/Unsqueeze_220",
            "/Unsqueeze_223",
            "/Unsqueeze_229",
            "/Unsqueeze_235",
            "/Unsqueeze_264",
            "/Unsqueeze_270",
            "/Unsqueeze_273",
            "/Unsqueeze_330",
            "/Unsqueeze_333",
            "/Unsqueeze_336",
            "/Unsqueeze_341",
            "/Unsqueeze_344",
        }:
            suffix = node.name.rsplit("_", 1)[-1]
            if node.name == "/Unsqueeze_201":
                batch_axes = "spark_axes1"
            elif node.name == "/Unsqueeze_229":
                batch_axes = "spark_axes1234"
            elif node.name in {"/Unsqueeze_146", "/Unsqueeze_165"}:
                batch_axes = "spark_axes123"
            else:
                batch_axes = "spark_axes12"
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Unsqueeze",
                        ["spark_batch_range", batch_axes],
                        [f"spark_batch_index_matrix_{suffix}"],
                    ),
                    helper.make_node(
                        "Shape", [node.input[0]], [f"spark_scatter_index_shape_{suffix}"]
                    ),
                    helper.make_node(
                        "Expand",
                        [
                            f"spark_batch_index_matrix_{suffix}",
                            f"spark_scatter_index_shape_{suffix}",
                        ],
                        [f"spark_scatter_batch_indexes_{suffix}"],
                    ),
                ]
            )
            node.input[0] = f"spark_scatter_batch_indexes_{suffix}"
        if node.name == "/Unsqueeze_190":
            node.input[0] = "spark_batch_range"
        if node.name == "/Unsqueeze_193":
            node.input[0] = "spark_batch_range"
        if node.name == "/mmm_net/ScatterElements_5":
            rewritten_nodes.extend(
                [
                    helper.make_node(
                        "Concat", ["spark_batch_vec", "spark_64"], ["spark_batch_64"], axis=0
                    ),
                    helper.make_node(
                        "Expand", [node.input[0], "spark_batch_64"], ["spark_token_mask_batched"]
                    ),
                    helper.make_node(
                        "Expand",
                        [node.input[2], "spark_batch_1"],
                        ["spark_token_mask_update_batched"],
                    ),
                ]
            )
            node.input[0] = "spark_token_mask_batched"
            node.input[2] = "spark_token_mask_update_batched"
        if node.name in {
            "/mmm_net/ScatterElements_7",
            "/mmm_net/ScatterElements_9",
            "/mmm_net/ScatterElements_11",
        }:
            suffix = node.name.rsplit("_", 1)[-1]
            rewritten_nodes.append(
                helper.make_node(
                    "Expand",
                    [node.input[2], "spark_batch_1"],
                    [f"spark_token_mask_update_batched_{suffix}"],
                )
            )
            node.input[2] = f"spark_token_mask_update_batched_{suffix}"
        rewritten_nodes.append(node)
    del model.graph.node[:]
    model.graph.node.extend(prefix + preamble + rewritten_nodes)
    return model


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert the Sonic V2 planner ONNX export to dynamic batch."
    )
    parser.add_argument("source")
    parser.add_argument("destination")
    args = parser.parse_args()
    onnx.save(transform(args.source), args.destination)
