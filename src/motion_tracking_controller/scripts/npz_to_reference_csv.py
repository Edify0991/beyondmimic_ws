#!/usr/bin/env python3
"""Convert BeyondMimic motion .npz to motion_tracking_controller local reference CSV.

Output format (one frame per line):
  joint_pos ; joint_vel ; body_pos ; body_quat

Where:
  - joint_pos: flattened joint positions (len = num_joints)
  - joint_vel: flattened joint velocities (len = num_joints)
  - body_pos: flattened xyz per body (len = 3 * num_bodies)
  - body_quat: flattened wxyz per body (len = 4 * num_bodies)
"""

from __future__ import annotations

import argparse
import pathlib
from typing import Iterable, List, Sequence

import numpy as np


def _pick_key(npz: np.lib.npyio.NpzFile, candidates: Sequence[str], field_name: str) -> str:
    for key in candidates:
        if key in npz:
            return key
    raise KeyError(
        f"Cannot find field '{field_name}'. Tried keys: {list(candidates)}. "
        f"Available keys: {list(npz.keys())}"
    )


def _to_scalar_fps(fps_arr: np.ndarray) -> float:
    fps_flat = np.asarray(fps_arr).reshape(-1)
    if fps_flat.size == 0:
        raise ValueError("fps exists but is empty.")
    fps = float(fps_flat[0])
    if fps <= 0:
        raise ValueError(f"fps must be > 0, got {fps}.")
    return fps


def _format_line(values: Iterable[float], precision: int) -> str:
    fmt = f"{{:.{precision}f}}"
    return ",".join(fmt.format(float(v)) for v in values)


def _parse_index_token(token: str) -> List[int]:
    token = token.strip()
    if not token:
        return []
    if "-" in token:
        parts = token.split("-")
        if len(parts) != 2:
            raise ValueError(f"Invalid range token: '{token}'")
        start = int(parts[0])
        end = int(parts[1])
        if end < start:
            raise ValueError(f"Invalid range token: '{token}' (end < start)")
        return list(range(start, end + 1))
    return [int(token)]


def _parse_body_indices(spec: str) -> List[int]:
    indices: List[int] = []
    for token in spec.split(","):
        indices.extend(_parse_index_token(token))
    if not indices:
        raise ValueError("body-indices is empty after parsing")
    # Keep order but remove duplicates.
    dedup: List[int] = []
    seen = set()
    for idx in indices:
        if idx not in seen:
            dedup.append(idx)
            seen.add(idx)
    return dedup


def _read_policy_body_names(onnx_path: pathlib.Path) -> List[str]:
    meta_body_names = ""
    try:
        import onnx  # type: ignore

        model = onnx.load(str(onnx_path), load_external_data=False)
        meta = {p.key: p.value for p in model.metadata_props}
        meta_body_names = str(meta.get("body_names", ""))
    except Exception:
        try:
            import onnxruntime as ort  # type: ignore

            session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
            meta = dict(session.get_modelmeta().custom_metadata_map)
            meta_body_names = str(meta.get("body_names", ""))
        except Exception as exc:
            raise RuntimeError(
                "--onnx-policy requested but neither Python package 'onnx' nor 'onnxruntime' is available. "
                "Install one of them, e.g. `pip install onnx`."
            ) from exc

    names = [x.strip() for x in meta_body_names.split(",") if x.strip()]
    if not names:
        raise RuntimeError("ONNX metadata 'body_names' is missing or empty.")
    return names


def _pick_optional_key(npz: np.lib.npyio.NpzFile, candidates: Sequence[str]) -> str | None:
    for key in candidates:
        if key in npz:
            return key
    return None


def _coerce_name_list(raw: np.ndarray | Sequence[object] | object) -> List[str]:
    arr = np.asarray(raw, dtype=object).reshape(-1)
    if arr.size == 0:
        return []
    # Some datasets store names as a single CSV string.
    if arr.size == 1 and isinstance(arr[0], str) and "," in arr[0]:
        return [x.strip() for x in arr[0].split(",") if x.strip()]
    out: List[str] = []
    for item in arr:
        text = str(item).strip()
        if text:
            out.append(text)
    return out


def _read_npz_body_names(npz: np.lib.npyio.NpzFile) -> List[str]:
    key = _pick_optional_key(
        npz,
        [
            "body_names",
            "rigid_body_names",
            "link_names",
            "body_name",
            "bodyName",
        ],
    )
    if key is None:
        return []
    return _coerce_name_list(np.asarray(npz[key], dtype=object))


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert BeyondMimic .npz motion to local reference CSV.")
    parser.add_argument("--input", required=True, help="Input .npz path")
    parser.add_argument("--output", required=True, help="Output .csv path")
    parser.add_argument("--start-frame", type=int, default=0, help="Start frame index (inclusive)")
    parser.add_argument("--end-frame", type=int, default=-1, help="End frame index (exclusive). -1 means to the end")
    parser.add_argument("--stride", type=int, default=1, help="Frame stride")
    parser.add_argument(
        "--quat-order",
        choices=["wxyz", "xyzw"],
        default="wxyz",
        help="Quaternion order in input npz body_quat field",
    )
    parser.add_argument(
        "--target-fps",
        type=float,
        default=0.0,
        help="Optional target FPS. >0 enables nearest-neighbor resampling (requires npz[fps]).",
    )
    parser.add_argument("--precision", type=int, default=6, help="Decimal places in CSV")
    parser.add_argument(
        "--body-count",
        type=int,
        default=0,
        help="If >0, keep the first N bodies from npz body tensors.",
    )
    parser.add_argument(
        "--body-indices",
        default="",
        help="Explicit body indices to keep, e.g. '0,1,2,5-8'. Overrides --onnx-policy and --body-count.",
    )
    parser.add_argument(
        "--onnx-policy",
        default="",
        help=(
            "Optional ONNX policy path. If provided, align bodies to ONNX metadata body_names. "
            "If input .npz has no body-name list and body count differs, provide --body-indices explicitly."
        ),
    )

    args = parser.parse_args()

    input_path = pathlib.Path(args.input).expanduser().resolve()
    output_path = pathlib.Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    data = np.load(input_path, allow_pickle=True)

    joint_pos_key = _pick_key(data, ["joint_pos", "dof_pos"], "joint_pos")
    joint_vel_key = _pick_key(data, ["joint_vel", "dof_vel"], "joint_vel")
    body_pos_key = _pick_key(data, ["body_pos_w", "body_pos"], "body_pos_w")
    body_quat_key = _pick_key(data, ["body_quat_w", "body_quat"], "body_quat_w")

    joint_pos = np.asarray(data[joint_pos_key], dtype=np.float64)
    joint_vel = np.asarray(data[joint_vel_key], dtype=np.float64)
    body_pos = np.asarray(data[body_pos_key], dtype=np.float64)
    body_quat = np.asarray(data[body_quat_key], dtype=np.float64)

    if joint_pos.ndim != 2:
        raise ValueError(f"{joint_pos_key} must be [T, J], got shape {joint_pos.shape}")
    if joint_vel.ndim != 2:
        raise ValueError(f"{joint_vel_key} must be [T, J], got shape {joint_vel.shape}")
    if body_pos.ndim != 3 or body_pos.shape[-1] != 3:
        raise ValueError(f"{body_pos_key} must be [T, B, 3], got shape {body_pos.shape}")
    if body_quat.ndim != 3 or body_quat.shape[-1] != 4:
        raise ValueError(f"{body_quat_key} must be [T, B, 4], got shape {body_quat.shape}")

    T = joint_pos.shape[0]
    if joint_vel.shape[0] != T or body_pos.shape[0] != T or body_quat.shape[0] != T:
        raise ValueError(
            "Frame count mismatch: "
            f"joint_pos={joint_pos.shape[0]}, joint_vel={joint_vel.shape[0]}, "
            f"body_pos={body_pos.shape[0]}, body_quat={body_quat.shape[0]}"
        )
    if joint_vel.shape[1] != joint_pos.shape[1]:
        raise ValueError(
            f"Joint dimension mismatch: joint_pos={joint_pos.shape[1]}, joint_vel={joint_vel.shape[1]}"
        )
    if body_quat.shape[1] != body_pos.shape[1]:
        raise ValueError(
            f"Body count mismatch: body_pos={body_pos.shape[1]}, body_quat={body_quat.shape[1]}"
        )

    start = max(0, args.start_frame)
    end = T if args.end_frame < 0 else min(T, args.end_frame)
    if start >= end:
        raise ValueError(f"Invalid frame range: start={start}, end={end}, total={T}")
    if args.stride <= 0:
        raise ValueError(f"stride must be > 0, got {args.stride}")

    indices = np.arange(start, end, args.stride, dtype=np.int64)
    if indices.size == 0:
        raise ValueError("No frames selected after applying start/end/stride.")

    if args.target_fps and args.target_fps > 0:
        if "fps" not in data:
            raise KeyError("target-fps requested but npz does not contain 'fps'.")
        src_fps = _to_scalar_fps(np.asarray(data["fps"]))
        ratio = float(args.target_fps) / src_fps
        out_count = max(1, int(round(indices.size * ratio)))
        remap = np.round(np.linspace(0, indices.size - 1, out_count)).astype(np.int64)
        indices = indices[remap]

    jp = joint_pos[indices]
    jv = joint_vel[indices]
    bp = body_pos[indices]
    bq = body_quat[indices]

    original_body_count = bp.shape[1]
    if args.body_indices:
        body_indices = _parse_body_indices(args.body_indices)
        min_idx = min(body_indices)
        max_idx = max(body_indices)
        if min_idx < 0 or max_idx >= original_body_count:
            raise ValueError(
                f"body-indices out of range. valid=[0,{original_body_count - 1}], got min={min_idx}, max={max_idx}."
            )
        bp = bp[:, body_indices, :]
        bq = bq[:, body_indices, :]
    elif args.onnx_policy:
        onnx_path = pathlib.Path(args.onnx_policy).expanduser().resolve()
        policy_body_names = _read_policy_body_names(onnx_path)
        policy_body_count = len(policy_body_names)
        if policy_body_count > original_body_count:
            raise ValueError(
                f"ONNX metadata has {policy_body_count} bodies, but npz only has {original_body_count}."
            )

        npz_body_names = _read_npz_body_names(data)
        if npz_body_names:
            name_to_index = {name: i for i, name in enumerate(npz_body_names)}
            missing = [name for name in policy_body_names if name not in name_to_index]
            if missing:
                raise ValueError(
                    "Input npz contains body names, but some ONNX body_names are missing: "
                    f"{missing}. Available npz body names: {npz_body_names}"
                )
            body_indices = [name_to_index[name] for name in policy_body_names]
            bp = bp[:, body_indices, :]
            bq = bq[:, body_indices, :]
            print("[info] Cropped/reordered bodies by ONNX body_names using npz body-name mapping.")
        elif original_body_count == policy_body_count:
            print("[info] npz body count already matches ONNX body_names count; keeping original body order.")
        else:
            raise ValueError(
                "Cannot infer body order: npz has no body-name list and body count differs from ONNX. "
                "Provide --body-indices in ONNX body_names order."
            )
    elif args.body_count > 0:
        if args.body_count > original_body_count:
            raise ValueError(
                f"body-count={args.body_count} exceeds available body count {original_body_count}."
            )
        bp = bp[:, : args.body_count, :]
        bq = bq[:, : args.body_count, :]

    if args.quat_order == "xyzw":
        # Convert input xyzw -> output wxyz
        bq = bq[..., [3, 0, 1, 2]]

    # Normalize quaternions to avoid numeric drift.
    norms = np.linalg.norm(bq, axis=-1, keepdims=True)
    norms = np.where(norms <= 1e-12, 1.0, norms)
    bq = bq / norms

    if args.onnx_policy:
        onnx_path = pathlib.Path(args.onnx_policy).expanduser().resolve()
        policy_body_count = len(_read_policy_body_names(onnx_path))
        if bp.shape[1] != policy_body_count:
            raise ValueError(
                f"Output body count {bp.shape[1]} does not match ONNX policy body_names count {policy_body_count}."
            )

    with output_path.open("w", encoding="utf-8") as f:
        f.write(f"# converted_from={input_path}\n")
        f.write(
            f"# frames={jp.shape[0]} joints={jp.shape[1]} bodies={bp.shape[1]} "
            f"(original_bodies={original_body_count})\n"
        )
        for i in range(jp.shape[0]):
            joint_pos_field = _format_line(jp[i].reshape(-1), args.precision)
            joint_vel_field = _format_line(jv[i].reshape(-1), args.precision)
            body_pos_field = _format_line(bp[i].reshape(-1), args.precision)
            body_quat_field = _format_line(bq[i].reshape(-1), args.precision)
            f.write(f"{joint_pos_field};{joint_vel_field};{body_pos_field};{body_quat_field}\n")

    print(f"[ok] input: {input_path}")
    print(f"[ok] output: {output_path}")
    print(f"[ok] frames: {jp.shape[0]}, joints: {jp.shape[1]}, bodies: {bp.shape[1]} (original={original_body_count})")


if __name__ == "__main__":
    main()
