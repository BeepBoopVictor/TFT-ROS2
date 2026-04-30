#!/usr/bin/env python3

import argparse
import inspect
import json
from pathlib import Path

import numpy as np
import torch

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def load_policy_from_checkpoint(checkpoint: str, dataset):
    """
    Carga robusta de políticas LeRobot 0.5.x.
    Para ACT, prioriza ACTPolicy.from_pretrained(), porque make_policy()
    puede construir la arquitectura sin cargar pesos entrenados.
    """
    checkpoint = Path(checkpoint)

    if not checkpoint.exists():
        raise FileNotFoundError(f"No existe checkpoint: {checkpoint}")

    config_path = checkpoint / "config.json"
    if not config_path.exists():
        raise FileNotFoundError(f"No existe config.json en: {checkpoint}")

    with open(config_path, "r") as f:
        policy_cfg_json = json.load(f)

    policy_type = policy_cfg_json.get("type", "")
    print("policy_type:", policy_type)

    if policy_type == "act":
        try:
            from lerobot.policies.act.modeling_act import ACTPolicy

            print("ACTPolicy.from_pretrained signature:", inspect.signature(ACTPolicy.from_pretrained))
            policy = ACTPolicy.from_pretrained(checkpoint)
            print("Política cargada mediante ACTPolicy.from_pretrained().")
            return policy
        except Exception as exc:
            print("[WARN] No se pudo cargar con ACTPolicy.from_pretrained():")
            print(repr(exc))

    try:
        from lerobot.policies.pretrained import PreTrainedPolicy

        print("PreTrainedPolicy.from_pretrained signature:", inspect.signature(PreTrainedPolicy.from_pretrained))
        policy = PreTrainedPolicy.from_pretrained(checkpoint)
        print("Política cargada mediante PreTrainedPolicy.from_pretrained().")
        return policy
    except Exception as exc:
        print("[WARN] No se pudo cargar con PreTrainedPolicy.from_pretrained():")
        print(repr(exc))

    try:
        from lerobot.policies.factory import make_policy
        from lerobot.configs.policies import PreTrainedConfig

        print("make_policy signature:", inspect.signature(make_policy))

        cfg = PreTrainedConfig.from_pretrained(checkpoint)
        kwargs = {}

        sig = inspect.signature(make_policy)
        if "ds_meta" in sig.parameters:
            kwargs["ds_meta"] = dataset.meta

        policy = make_policy(cfg, **kwargs)
        print("[WARN] Política creada mediante make_policy(), pero puede no contener pesos entrenados.")
        return policy

    except Exception as exc:
        print("[WARN] No se pudo cargar con make_policy():")
        print(repr(exc))

    raise RuntimeError("No se pudo cargar la política desde el checkpoint.")


def prepare_batch(sample, device: str):
    batch = {}

    for key, value in sample.items():
        if key == "task":
            batch[key] = value
            continue

        if torch.is_tensor(value):
            batch[key] = value.unsqueeze(0).to(device)
        else:
            batch[key] = value

    return batch


def infer_action(policy, sample, device: str) -> np.ndarray:
    """
    Devuelve siempre np.ndarray con shape [action_dim].
    """
    policy.eval()
    policy.to(device)

    if hasattr(policy, "reset"):
        policy.reset()

    batch = prepare_batch(sample, device)

    with torch.no_grad():
        if hasattr(policy, "select_action"):
            action = policy.select_action(batch)
        elif hasattr(policy, "predict_action"):
            action = policy.predict_action(batch)
        else:
            output = policy(batch)
            if isinstance(output, dict):
                action = output.get("action", output.get("actions"))
            else:
                action = output

    if action is None:
        raise RuntimeError("No se pudo obtener acción de la política.")

    if isinstance(action, dict):
        action = action.get("action", action.get("actions"))

    if action is None:
        raise RuntimeError("La política devolvió un diccionario sin 'action' ni 'actions'.")

    if isinstance(action, np.ndarray):
        action_np = action
    elif torch.is_tensor(action):
        action_np = action.detach().cpu().numpy()
    else:
        raise RuntimeError(f"Tipo de acción inesperado: {type(action)}")

    action_np = np.asarray(action_np)

    if action_np.ndim == 3:
        action_np = action_np[0, 0]
    elif action_np.ndim == 2:
        action_np = action_np[0]
    elif action_np.ndim == 1:
        pass
    else:
        raise RuntimeError(f"Shape de acción inesperada: {action_np.shape}")

    return action_np.astype(np.float32)


def tensor_to_numpy(value) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value.squeeze()

    if torch.is_tensor(value):
        return value.detach().cpu().numpy().squeeze()

    return np.asarray(value).squeeze()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--repo-id", default="tfg/fp3_pick_place")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()

    device = args.device
    if device == "cuda" and not torch.cuda.is_available():
        print("[WARN] CUDA no disponible. Usando CPU.")
        device = "cpu"

    dataset = LeRobotDataset(
        repo_id=args.repo_id,
        root=args.dataset_root,
        download_videos=False,
    )

    print("=== DATASET ===")
    print("num_frames:", dataset.num_frames)
    print("num_episodes:", dataset.num_episodes)
    print("features:", dataset.features)

    print("")
    print("=== CARGANDO POLÍTICA ===")
    policy = load_policy_from_checkpoint(args.checkpoint, dataset)
    policy.to(device)
    policy.eval()

    sample = dataset[args.index]

    print("")
    print("=== SAMPLE KEYS ===")
    for k, v in sample.items():
        if isinstance(v, torch.Tensor):
            print(k, tuple(v.shape), v.dtype)
        else:
            print(k, type(v))

    pred_np = infer_action(policy, sample, device)
    gt_np = tensor_to_numpy(sample["action"]).astype(np.float32)

    if pred_np.shape != gt_np.shape:
        raise RuntimeError(
            f"Shape incompatible entre predicción y GT: pred={pred_np.shape}, gt={gt_np.shape}"
        )

    abs_error = np.abs(pred_np - gt_np)

    print("")
    print("=== INFERENCIA OFFLINE ===")
    print("index:", args.index)
    print("device:", device)
    print("gt_action:")
    print(np.array2string(gt_np, precision=4, suppress_small=True))
    print("pred_action:")
    print(np.array2string(pred_np, precision=4, suppress_small=True))
    print("abs_error:")
    print(np.array2string(abs_error, precision=4, suppress_small=True))

    print("")
    print("=== INTERPRETACIÓN ===")
    names = dataset.features["action"]["names"]
    for name, gt, pred, err in zip(names, gt_np, pred_np, abs_error):
        print(f"{name:24s} gt={gt: .5f}  pred={pred: .5f}  abs_err={err: .5f}")

    print("")
    print("=== VALIDACIÓN DE RANGOS FÍSICOS ===")

    x, y, z, qx, qy, qz, qw, gripper = pred_np.tolist()

    checks = {
        "x_in_range": 0.00 <= x <= 0.85,
        "y_in_range": -0.70 <= y <= 0.70,
        "z_in_range": 0.15 <= z <= 0.60,
        "gripper_in_range": 0.008 <= gripper <= 0.040,
    }

    for name, ok in checks.items():
        print(f"{name:20s}: {ok}")

    if not all(checks.values()):
        print("")
        print("[WARN] La acción predicha contiene valores fuera del rango seguro.")
        print("[WARN] No ejecutes todavía esta salida directamente en Gazebo sin clipping/safety layer.")
    else:
        print("")
        print("[OK] La acción predicha está dentro de rangos físicos razonables.")


if __name__ == "__main__":
    main()