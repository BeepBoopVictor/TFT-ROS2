#!/usr/bin/env python3
"""
act_policy_server.py — Servidor ZMQ de la policy ACT (lerobot_venv, Python 3.12).

Uso:
  use-il
  python act_policy_server.py \
      --policy-path  /root/.../pretrained_model \
      --dataset-root /root/.../fp3_pick_place_lerobot
"""

import argparse
import random
import sys
import time

import numpy as np
import torch
import zmq

try:
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError as e:
    print(f"[SERVER][FATAL] {e}")
    sys.exit(1)


def pack_array(arr: np.ndarray) -> dict:
    return {"shape": list(arr.shape), "dtype": str(arr.dtype), "data": arr.tobytes()}

def unpack_array(d) -> np.ndarray:
    """Acepta: numpy array directo, dict {shape,dtype,bytes/data/buffer}, o lista."""
    if isinstance(d, np.ndarray):
        return d.copy()
    if isinstance(d, dict):
        shape = d.get("shape")
        dtype = d.get("dtype", "float32")
        buf = None
        for key in ("bytes", "data", "buffer", "b"):
            if key in d:
                buf = d[key]; break
        if buf is not None and shape is not None:
            return np.frombuffer(buf, dtype=np.dtype(dtype)).reshape(shape).copy()
        raise KeyError(f"unpack_array: keys={list(d.keys())}, no encuentro buffer")
    return np.asarray(d, dtype=np.float32)


def compute_dataset_stats(dataset_root: str, repo_id: str, n_samples: int = 2000):
    print(f"[SERVER] Computando stats del dataset ({n_samples} samples)...")
    ds = LeRobotDataset(repo_id=repo_id, root=dataset_root)
    random.seed(42)
    n = min(n_samples, ds.num_frames)
    indices = sorted(random.sample(range(ds.num_frames), n))
    states, actions = [], []
    for i in indices:
        item = ds[i]
        s = item["observation.state"]
        a = item["action"]
        if torch.is_tensor(s): s = s.cpu().numpy()
        if torch.is_tensor(a): a = a.cpu().numpy()
        states.append(np.asarray(s, dtype=np.float32))
        actions.append(np.asarray(a, dtype=np.float32))
    states = np.stack(states)
    actions = np.stack(actions)
    return {
        "state_mean": states.mean(0).astype(np.float32),
        "state_std": np.maximum(states.std(0), 1e-6).astype(np.float32),
        "action_mean": actions.mean(0).astype(np.float32),
        "action_std": np.maximum(actions.std(0), 1e-6).astype(np.float32),
    }


class ACTPolicyServer:
    def __init__(self, args):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        print(f"[SERVER] cargando policy desde {args.policy_path} en {self.device} ...")
        self.policy = ACTPolicy.from_pretrained(args.policy_path)
        self.policy.eval().to(self.device)
        self.chunk_size = int(self.policy.config.chunk_size)
        self.n_action_steps = int(self.policy.config.n_action_steps)
        print(f"[SERVER] policy lista. chunk_size={self.chunk_size} n_action_steps={self.n_action_steps}")

        stats = compute_dataset_stats(args.dataset_root, args.repo_id)
        self.state_mean  = torch.tensor(stats["state_mean"],  device=self.device)
        self.state_std   = torch.tensor(stats["state_std"],   device=self.device)
        self.action_mean = torch.tensor(stats["action_mean"], device=self.device)
        self.action_std  = torch.tensor(stats["action_std"],  device=self.device)
        self.img_mean = torch.tensor([0.485, 0.456, 0.406], device=self.device).reshape(1, 3, 1, 1)
        self.img_std  = torch.tensor([0.229, 0.224, 0.225], device=self.device).reshape(1, 3, 1, 1)

        print(f"[SERVER] state_mean: {np.round(stats['state_mean'], 4).tolist()}")
        print(f"[SERVER] action_mean: {np.round(stats['action_mean'], 4).tolist()}")

        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.REP)
        self.sock.bind(args.bind)
        print(f"[SERVER] escuchando en {args.bind}. Ctrl+C para salir.")
        self.step = 0
        self.grip_closed = False
        self.grip_open_count = 0

        # Logging para TFG
        self.log_dir = args.log_dir
        self.save_frames_every = args.save_frames_every if hasattr(args, 'save_frames_every') else 3
        self.csv_file = None
        self.csv_writer = None
        if self.log_dir:
            import os, csv
            os.makedirs(self.log_dir, exist_ok=True)
            os.makedirs(os.path.join(self.log_dir, "frames_top"), exist_ok=True)
            os.makedirs(os.path.join(self.log_dir, "frames_cab"), exist_ok=True)
            csv_path = os.path.join(self.log_dir, "trajectory.csv")
            self.csv_file = open(csv_path, "w", newline="")
            header = (["step", "timestamp", "infer_ms"] +
                      [f"state_j{i}" for i in range(1, 8)] + ["state_grip"] +
                      [f"action_j{i}" for i in range(1, 8)] + ["action_grip_raw", "action_grip_out"])
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(header)
            print(f"[SERVER] logging a {self.log_dir}")

    def handle_ping(self):
        return {"ok": True, "device": str(self.device),
                "chunk_size": self.chunk_size, "n_action_steps": self.n_action_steps}

    def handle_reset(self):
        self.policy.reset()
        self.step = 0
        self.grip_closed = False
        self.grip_open_count = 0
        print("[SERVER] policy.reset()")
        return {"ok": True}

    def handle_infer(self, msg):
        t0 = time.time()
        self.step += 1

        if self.step == 1:
            print(f"[SERVER][DEBUG] msg keys: {[k for k in msg.keys() if k != 'cmd']}")
            for k, v in msg.items():
                if k == "cmd":
                    continue
                if isinstance(v, dict):
                    print(f"[SERVER][DEBUG]   '{k}': dict keys={list(v.keys())}")
                elif isinstance(v, np.ndarray):
                    print(f"[SERVER][DEBUG]   '{k}': ndarray {v.shape} {v.dtype}")
                elif v is None:
                    print(f"[SERVER][DEBUG]   '{k}': None")
                else:
                    print(f"[SERVER][DEBUG]   '{k}': {type(v).__name__}")

        state_raw = msg.get("state")
        top_raw = msg.get("top") or msg.get("top_image") or msg.get("image_top")
        cab_raw = (msg.get("cab") or msg.get("cabinet") or
                   msg.get("cab_image") or msg.get("image_cabinet") or
                   msg.get("cabinet_image"))

        if state_raw is None:
            return {"ok": False, "error": "falta 'state' en el mensaje"}

        state = unpack_array(state_raw)

        if top_raw is not None:
            top = unpack_array(top_raw)
        else:
            top = np.zeros((224, 224, 3), dtype=np.uint8)
            if self.step <= 2:
                print("[SERVER][WARN] imagen top no encontrada, usando fallback negro")

        if cab_raw is not None:
            cab = unpack_array(cab_raw)
        else:
            cab = top.copy()
            if self.step <= 2:
                print("[SERVER][WARN] imagen cabinet no encontrada, usando top como fallback")

        top_t   = torch.from_numpy(top).permute(2, 0, 1).unsqueeze(0).to(self.device).float() / 255.0
        cab_t   = torch.from_numpy(cab).permute(2, 0, 1).unsqueeze(0).to(self.device).float() / 255.0
        state_t = torch.from_numpy(state).unsqueeze(0).to(self.device).float()

        batch = {
            "observation.state":          (state_t - self.state_mean) / self.state_std,
            "observation.images.top":     (top_t - self.img_mean) / self.img_std,
            "observation.images.cabinet": (cab_t - self.img_mean) / self.img_std,
        }

        with torch.no_grad():
            pred_norm = self.policy.select_action(batch)
        if pred_norm.dim() == 2:
            pred_norm = pred_norm.squeeze(0)

        action = (pred_norm * self.action_std + self.action_mean).cpu().numpy().astype(np.float32)

        # Gripper: binarizado.
        # - Cierra cuando grip < 0.4195 
        # - Abre cuando grip > 0.9
        grip = float(action[7])
        if not self.grip_closed:
            if grip < 0.4195:
                self.grip_closed = True
                self.grip_open_count = 0
        else:
            if grip > 0.9:
                self.grip_open_count += 1
                if self.grip_open_count >= 3:
                    self.grip_closed = False
                    self.grip_open_count = 0
            else:
                self.grip_open_count = 0
        action[7] = 0.0 if self.grip_closed else 1.0

        dt = (time.time() - t0) * 1000.0
        if self.step <= 5 or self.step % 25 == 0:
            print(f"[SERVER] step={self.step:4d} infer={dt:5.1f}ms "
                  f"arm[:3]={action[:3]} grip_raw={grip:.2f} grip_out={float(action[7]):.0f}")

        if self.csv_writer is not None:
            import cv2
            row = ([self.step, f"{time.time():.3f}", f"{dt:.1f}"] +
                   [f"{state[i]:.6f}" for i in range(7)] + [f"{state[7]:.4f}"] +
                   [f"{action[i]:.6f}" for i in range(7)] + [f"{grip:.4f}", f"{action[7]:.0f}"])
            self.csv_writer.writerow(row)
            self.csv_file.flush()
            if self.step % self.save_frames_every == 0:
                top_bgr = cv2.cvtColor(top, cv2.COLOR_RGB2BGR)
                cab_bgr = cv2.cvtColor(cab, cv2.COLOR_RGB2BGR)
                cv2.imwrite(f"{self.log_dir}/frames_top/step_{self.step:05d}.jpg", top_bgr)
                cv2.imwrite(f"{self.log_dir}/frames_cab/step_{self.step:05d}.jpg", cab_bgr)

        return {"ok": True, "action": pack_array(action)}

    def run(self):
        while True:
            try:
                raw = self.sock.recv_pyobj()
            except KeyboardInterrupt:
                print("\n[SERVER] saliendo...")
                break
            cmd = raw.get("cmd", "")
            try:
                if cmd == "ping":    resp = self.handle_ping()
                elif cmd == "reset": resp = self.handle_reset()
                elif cmd == "infer": resp = self.handle_infer(raw)
                else:                resp = {"ok": False, "error": f"cmd desconocido: {cmd}"}
            except Exception as exc:
                import traceback; traceback.print_exc()
                resp = {"ok": False, "error": str(exc)}
            self.sock.send_pyobj(resp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy-path", required=True)
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--repo-id", default="tfg/fp3_pick_place_act")
    ap.add_argument("--bind", default="tcp://127.0.0.1:5555")
    ap.add_argument("--log-dir", default=None,
                    help="Si se especifica, guarda CSV de trayectorias y frames de camaras "
                         "para generar graficas y video del TFG.")
    ap.add_argument("--save-frames-every", type=int, default=3,
                    help="Guardar frames de camara cada N steps (default 3)")
    main_args = ap.parse_args()
    ACTPolicyServer(main_args).run()


if __name__ == "__main__":
    main()
