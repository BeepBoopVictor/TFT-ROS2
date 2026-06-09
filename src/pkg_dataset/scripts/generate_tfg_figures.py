#!/usr/bin/env python3
"""
generate_tfg_figures_v2.py

Lee los logs generados por act_policy_server.py (--log-dir) y produce material visual
para la memoria del TFG:
  1. Trayectorias articulares (7 joints) a lo largo del episodio
  2. Estado del gripper (predicción raw + salida binaria)  [conservada]
  3. Variación/suavidad de la acción articular por step     [sustituye a latencia]
  4. Resumen numérico del episodio
  5. Vídeo side-by-side de ambas cámaras, con cabecera y pie separados

Uso:
  python generate_tfg_figures_v2.py --log-dir /root/tfg_panda_ws/logs/demo_run
"""

import argparse
import csv
import glob
import os
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("[FATAL] pip install matplotlib")
    sys.exit(1)

try:
    import cv2
except ImportError:
    print("[WARN] cv2 no disponible, no se generara video")
    cv2 = None


JOINT_NAMES = [f"J{i}" for i in range(1, 8)]
HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float32)

C = {
    "state": "#1565C0",
    "action": "#EF6C00",
    "home": "#616161",
    "gripper": "#7B1FA2",
    "open": "#2E7D32",
    "close": "#C62828",
    "smooth": "#00838F",
    "grid": "#9E9E9E",
}


def load_csv(path):
    steps, times, infer_ms = [], [], []
    states, actions = [], []
    grip_raw, grip_out = [], []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            steps.append(int(row["step"]))
            times.append(float(row["timestamp"]))
            infer_ms.append(float(row.get("infer_ms", 0.0)))
            states.append([float(row[f"state_j{i}"]) for i in range(1, 8)])
            actions.append([float(row[f"action_j{i}"]) for i in range(1, 8)])
            grip_raw.append(float(row["action_grip_raw"]))
            grip_out.append(float(row["action_grip_out"]))

    if not steps:
        raise RuntimeError(f"CSV vacío: {path}")

    t0 = times[0]
    times_rel = [t - t0 for t in times]
    return {
        "steps": np.array(steps),
        "time": np.array(times_rel),
        "infer_ms": np.array(infer_ms),
        "states": np.array(states),
        "actions": np.array(actions),
        "grip_raw": np.array(grip_raw),
        "grip_out": np.array(grip_out),
    }


def moving_average(y, window=5):
    y = np.asarray(y, dtype=float)
    if len(y) == 0 or window <= 1:
        return y
    window = min(int(window), len(y))
    kernel = np.ones(window, dtype=float) / window
    pad_left = window // 2
    pad_right = window - 1 - pad_left
    y_pad = np.pad(y, (pad_left, pad_right), mode="edge")
    return np.convolve(y_pad, kernel, mode="valid")


def plot_joint_trajectories(data, out_path):
    """Trayectorias articulares: estado actual vs acción comandada."""
    fig, axes = plt.subplots(4, 2, figsize=(14, 10), sharex=True)
    axes = axes.flatten()
    t = data["time"]

    for j in range(7):
        ax = axes[j]
        ax.plot(t, data["states"][:, j], "-", linewidth=1.3,
                label="Estado actual", color=C["state"])
        ax.plot(t, data["actions"][:, j], "--", linewidth=0.95,
                label="Acción comandada", color=C["action"], alpha=0.85)
        ax.axhline(HOME[j], color=C["home"], linestyle=":", linewidth=0.8,
                   alpha=0.65, label="Home" if j == 0 else None)
        ax.set_ylabel(f"J{j+1} (rad)", fontsize=9)
        ax.tick_params(labelsize=8)
        ax.grid(True, alpha=0.25)
        if j == 0:
            ax.legend(fontsize=7, loc="upper right")

    ax = axes[7]
    ax.plot(t, data["grip_raw"], "-", linewidth=1.2,
            label="Predicción raw", color=C["gripper"])
    ax.step(t, data["grip_out"], where="post", linewidth=1.4,
            label="Salida binaria", color=C["open"])
    ax.fill_between(t, 0, data["grip_out"], alpha=0.18, color=C["open"])
    ax.set_ylabel("Gripper", fontsize=9)
    ax.set_xlabel("Tiempo (s)", fontsize=9)
    ax.set_ylim(-0.1, 1.1)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.25)

    fig.suptitle("Trayectorias articulares durante Pick-and-Place con ACT",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_path, dpi=220, bbox_inches="tight")
    fig.savefig(str(out_path).replace(".png", ".svg"), bbox_inches="tight")
    plt.close(fig)
    print(f"[FIG] Trayectorias guardadas en {out_path}")


def plot_gripper_detail(data, out_path):
    """Detalle del gripper con histéresis. Se conserva porque es útil para el TFG."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 5), sharex=True)
    t = data["time"]

    ax1.plot(t, data["grip_raw"], "-", linewidth=1.3,
             color=C["gripper"], label="Predicción policy (raw)")
    ax1.axhline(0.4195, color=C["close"], linestyle="--", linewidth=1.0,
                alpha=0.75, label="Umbral de cierre")
    ax1.axhline(0.90, color=C["open"], linestyle="--", linewidth=1.0,
                alpha=0.75, label="Umbral de reapertura")
    ax1.set_ylabel("grip_raw", fontsize=10)
    ax1.legend(fontsize=8, loc="best")
    ax1.grid(True, alpha=0.25)
    ax1.set_title("Control del gripper", fontsize=11, fontweight="bold")

    ax2.fill_between(t, 0, data["grip_out"], alpha=0.28, color=C["open"])
    ax2.step(t, data["grip_out"], where="post", linewidth=1.6,
             color=C["open"], label="Comando al robot")
    ax2.set_ylabel("Gripper\n0=cerrado, 1=abierto", fontsize=10)
    ax2.set_xlabel("Tiempo (s)", fontsize=10)
    ax2.set_ylim(-0.1, 1.1)
    ax2.legend(fontsize=8, loc="best")
    ax2.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_path, dpi=220, bbox_inches="tight")
    fig.savefig(str(out_path).replace(".png", ".svg"), bbox_inches="tight")
    plt.close(fig)
    print(f"[FIG] Gripper detalle guardado en {out_path}")


def plot_action_smoothness(data, out_path):
    """
    Sustituye la figura de latencia por una métrica más interpretable para la memoria:
    cuánto cambia la acción articular comandada entre steps consecutivos.
    """
    t = data["time"]
    actions = data["actions"]

    if len(actions) < 2:
        print("[WARN] No hay suficientes steps para action_smoothness")
        return

    dq_cmd = np.linalg.norm(np.diff(actions, axis=0), axis=1)
    t_mid = t[1:]
    dq_smooth = moving_average(dq_cmd, window=7)

    fig, ax = plt.subplots(figsize=(12, 4.2))
    ax.plot(t_mid, dq_cmd, linewidth=0.8, alpha=0.30, color=C["action"],
            label="Cambio instantáneo")
    ax.plot(t_mid, dq_smooth, linewidth=2.2, color=C["smooth"],
            label="Media móvil")
    ax.axhline(np.median(dq_cmd), color=C["home"], linestyle="--", linewidth=1.1,
               label=f"Mediana: {np.median(dq_cmd):.3f} rad")

    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel(r"$||a_t - a_{t-1}||_2$ (rad)")
    ax.set_title("Suavidad de la acción articular comandada por ACT",
                 fontsize=11, fontweight="bold")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=9, loc="best")

    fig.tight_layout()
    fig.savefig(out_path, dpi=220, bbox_inches="tight")
    fig.savefig(str(out_path).replace(".png", ".svg"), bbox_inches="tight")
    plt.close(fig)
    print(f"[FIG] Suavidad de acción guardada en {out_path}")


def write_episode_summary(data, out_path):
    """Genera un TXT con métricas simples para acompañar las figuras."""
    duration = float(data["time"][-1]) if len(data["time"]) else 0.0
    n_steps = int(len(data["steps"]))
    actions = data["actions"]
    states = data["states"]

    dq_cmd = np.linalg.norm(np.diff(actions, axis=0), axis=1) if len(actions) > 1 else np.array([0.0])
    movement = np.linalg.norm(states - HOME.reshape(1, -1), axis=1)
    n_closed = int(np.sum(data["grip_out"] == 0.0))
    n_open = int(np.sum(data["grip_out"] == 1.0))

    lines = [
        "Resumen del episodio ACT\n",
        f"Steps registrados: {n_steps}\n",
        f"Duración aproximada: {duration:.2f} s\n",
        f"Latencia mediana: {np.median(data['infer_ms']):.2f} ms\n",
        f"Latencia P95: {np.percentile(data['infer_ms'], 95):.2f} ms\n",
        f"Cambio articular comandado medio: {np.mean(dq_cmd):.4f} rad\n",
        f"Cambio articular comandado máximo: {np.max(dq_cmd):.4f} rad\n",
        f"Separación media respecto a HOME: {np.mean(movement):.4f} rad\n",
        f"Steps con gripper abierto: {n_open}\n",
        f"Steps con gripper cerrado: {n_closed}\n",
    ]
    Path(out_path).write_text("".join(lines), encoding="utf-8")
    print(f"[TXT] Resumen guardado en {out_path}")


def _draw_label_box(img, text, org, font_scale=0.55, color=(255, 255, 255), bg=(30, 30, 30)):
    """Texto con fondo semitransparente para mejorar legibilidad."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = org
    pad = 6
    x1, y1 = max(0, x - pad), max(0, y - th - pad)
    x2, y2 = min(img.shape[1] - 1, x + tw + pad), min(img.shape[0] - 1, y + baseline + pad)
    overlay = img.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), bg, -1)
    cv2.addWeighted(overlay, 0.70, img, 0.30, 0, img)
    cv2.putText(img, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)


def make_video(log_dir, out_path, fps=10, data=None):
    """Crea vídeo side-by-side de ambas cámaras sin texto encima de las imágenes."""
    if cv2 is None:
        print("[WARN] cv2 no disponible, no se genera video")
        return

    top_frames = sorted(glob.glob(os.path.join(log_dir, "frames_top", "*.jpg")))
    cab_frames = sorted(glob.glob(os.path.join(log_dir, "frames_cab", "*.jpg")))

    if not top_frames:
        print("[WARN] No hay frames guardados en frames_top/")
        return

    n = min(len(top_frames), len(cab_frames))
    if n == 0:
        print("[WARN] No hay frames emparejados top/cabinet")
        return

    t0 = cv2.imread(top_frames[0])
    c0 = cv2.imread(cab_frames[0])
    if t0 is None or c0 is None:
        print("[WARN] No se puede leer el primer frame")
        return

    h, w = t0.shape[:2]
    gap = 12
    header = 42
    footer = 46
    total_w = w * 2 + gap
    total_h = header + h + footer

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, fps, (total_w, total_h))

    step_to_time = {}
    step_to_grip = {}
    if data is not None:
        step_to_time = {int(s): float(t) for s, t in zip(data["steps"], data["time"])}
        step_to_grip = {int(s): float(g) for s, g in zip(data["steps"], data["grip_out"])}

    for i in range(n):
        top = cv2.imread(top_frames[i])
        cab = cv2.imread(cab_frames[i])
        if top is None or cab is None:
            continue

        if top.shape[:2] != (h, w):
            top = cv2.resize(top, (w, h))
        if cab.shape[:2] != (h, w):
            cab = cv2.resize(cab, (w, h))

        canvas = np.full((total_h, total_w, 3), 20, dtype=np.uint8)
        canvas[header:header + h, :w] = top
        canvas[header:header + h, w + gap:w + gap + w] = cab

        _draw_label_box(canvas, "Camera Top (Conveyor)", (12, 27), font_scale=0.58)
        _draw_label_box(canvas, "Camera Cabinet", (w + gap + 12, 27), font_scale=0.58)

        cv2.line(canvas, (w + gap // 2, header), (w + gap // 2, header + h), (70, 70, 70), 1)
        cv2.line(canvas, (0, header + h), (total_w, header + h), (70, 70, 70), 1)

        step_str = os.path.basename(top_frames[i]).replace("step_", "").replace(".jpg", "")
        try:
            step_num = int(step_str)
        except ValueError:
            step_num = -1

        time_txt = f"t={step_to_time.get(step_num, i / max(fps, 1)):.2f}s"
        grip_val = step_to_grip.get(step_num, None)
        grip_txt = "gripper=n/d" if grip_val is None else ("gripper=abierto" if grip_val >= 0.5 else "gripper=cerrado")
        footer_text = f"Step {step_str}  |  {time_txt}  |  {grip_txt}"

        (tw, _), _ = cv2.getTextSize(footer_text, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 1)
        x = max(12, (total_w - tw) // 2)
        cv2.putText(canvas, footer_text, (x, total_h - 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 230, 255), 1, cv2.LINE_AA)

        writer.write(canvas)

    writer.release()
    print(f"[VIDEO] Video guardado en {out_path} ({n} frames, {fps} fps)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir", required=True,
                    help="Directorio con trajectory.csv y frames_top/ frames_cab/")
    ap.add_argument("--video-fps", type=int, default=10)
    ap.add_argument("--skip-video", action="store_true",
                    help="No genera el MP4, solo figuras y resumen")
    args = ap.parse_args()

    csv_path = os.path.join(args.log_dir, "trajectory.csv")
    if not os.path.exists(csv_path):
        print(f"[FATAL] No existe {csv_path}")
        sys.exit(1)

    data = load_csv(csv_path)
    print(f"[INFO] Cargados {len(data['steps'])} steps, duracion={data['time'][-1]:.1f}s")

    out = os.path.join(args.log_dir, "figures")
    os.makedirs(out, exist_ok=True)

    plot_joint_trajectories(data, os.path.join(out, "joint_trajectories.png"))
    plot_gripper_detail(data, os.path.join(out, "gripper_detail.png"))
    plot_action_smoothness(data, os.path.join(out, "action_smoothness.png"))
    write_episode_summary(data, os.path.join(out, "episode_summary.txt"))

    if not args.skip_video:
        make_video(args.log_dir, os.path.join(out, "demo_dual_camera.mp4"), args.video_fps, data=data)

    print(f"\n[DONE] Todas las salidas en {out}/")
    print("  - joint_trajectories.png / .svg")
    print("  - gripper_detail.png / .svg")
    print("  - action_smoothness.png / .svg")
    print("  - episode_summary.txt")
    if not args.skip_video:
        print("  - demo_dual_camera.mp4")


if __name__ == "__main__":
    main()
