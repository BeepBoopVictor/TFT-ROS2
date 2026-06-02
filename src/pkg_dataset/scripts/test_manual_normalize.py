#!/usr/bin/env python3
"""
test_manual_normalize.py — Prueba si normalizar manualmente inputs/outputs
arregla las predicciones de la policy (bypass del bug de stats no baked).
"""
import random, sys
import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy

POLICY_PATH = "/root/tfg_panda_ws/outputs/train/act_fp3_pick_place_v2/checkpoints/last/pretrained_model"
DATASET_ROOT = "/root/tfg_panda_ws/datasets/fp3_pick_place_lerobot"
REPO_ID = "tfg/fp3_pick_place_act"

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 1. Cargar
print("[TEST] Cargando dataset y policy...")
ds = LeRobotDataset(repo_id=REPO_ID, root=DATASET_ROOT)
policy = ACTPolicy.from_pretrained(POLICY_PATH)
policy.eval().to(device)

# 2. Computar stats del dataset
random.seed(42)
n = min(2000, ds.num_frames)
indices = sorted(random.sample(range(ds.num_frames), n))
states, actions = [], []
for i in indices:
    item = ds[i]
    states.append(item["observation.state"].cpu().numpy())
    actions.append(item["action"].cpu().numpy())
states = np.stack(states); actions = np.stack(actions)

s_mean = torch.tensor(states.mean(0), dtype=torch.float32, device=device)
s_std  = torch.tensor(np.maximum(states.std(0), 1e-6), dtype=torch.float32, device=device)
a_mean = torch.tensor(actions.mean(0), dtype=torch.float32, device=device)
a_std  = torch.tensor(np.maximum(actions.std(0), 1e-6), dtype=torch.float32, device=device)

# ImageNet stats (lo que la policy usaria si el normalize funcionara)
img_mean = torch.tensor([0.485, 0.456, 0.406], device=device).reshape(1, 3, 1, 1)
img_std  = torch.tensor([0.229, 0.224, 0.225], device=device).reshape(1, 3, 1, 1)

print(f"[TEST] state_mean: {np.round(s_mean.cpu().numpy(), 4).tolist()}")
print(f"[TEST] state_std:  {np.round(s_std.cpu().numpy(), 4).tolist()}")

# 3. Test: SIN normalizar (como esta ahora, deberia dar error ~1.4)
print("\n=== SIN normalizacion manual (status quo) ===")
for i in [0, 50, 100]:
    if i >= ds.num_frames: break
    item = ds[i]
    batch = {
        "observation.state": item["observation.state"].unsqueeze(0).to(device).float(),
        "observation.images.top": item["observation.images.top"].unsqueeze(0).to(device).float(),
        "observation.images.cabinet": item["observation.images.cabinet"].unsqueeze(0).to(device).float(),
    }
    policy.reset()
    with torch.no_grad():
        pred = policy.select_action(batch)
    if pred.dim() == 2: pred = pred.squeeze(0)
    gt = item["action"].cpu().numpy()
    pred_np = pred.cpu().numpy()
    print(f"  frame {i}: err={np.linalg.norm(pred_np - gt):.4f}")

# 4. Test: CON normalizacion manual (deberia dar error < 0.3 si esta es la causa)
print("\n=== CON normalizacion manual (el fix) ===")
errors = []
for i in [0, 10, 30, 50, 100, 200, 400]:
    if i >= ds.num_frames: break
    item = ds[i]
    state = item["observation.state"].unsqueeze(0).to(device).float()
    top   = item["observation.images.top"].unsqueeze(0).to(device).float()
    cab   = item["observation.images.cabinet"].unsqueeze(0).to(device).float()
    gt    = item["action"].cpu().numpy()

    # Normalizar manualmente
    state_norm = (state - s_mean) / s_std
    top_norm   = (top - img_mean) / img_std
    cab_norm   = (cab - img_mean) / img_std

    batch = {
        "observation.state": state_norm,
        "observation.images.top": top_norm,
        "observation.images.cabinet": cab_norm,
    }
    policy.reset()
    with torch.no_grad():
        pred_norm = policy.select_action(batch)
    if pred_norm.dim() == 2: pred_norm = pred_norm.squeeze(0)

    # Desnormalizar action manualmente
    pred_raw = (pred_norm.cpu() * a_std.cpu() + a_mean.cpu()).numpy()

    err = float(np.linalg.norm(pred_raw - gt))
    errors.append(err)
    print(f"  frame {i}: err={err:.4f}  gt[:3]={np.round(gt[:3], 3)}  pred[:3]={np.round(pred_raw[:3], 3)}")

print(f"\n  Error medio CON normalize manual: {np.mean(errors):.4f}")
if np.mean(errors) < 0.3:
    print("  -> FUNCIONA. Incorporar normalizacion manual al act_policy_server.py.")
elif np.mean(errors) < 1.0:
    print("  -> MEJORA parcial. Puede ser que ImageNet normalize ya se aplique internamente.")
    print("     Probar sin normalizar imagenes (solo state/action).")
else:
    print("  -> NO FUNCIONA. El bug es mas profundo.")
