#!/bin/bash
# train_act.sh

# Uso:
#   use-il
#   bash src/pkg_dataset/scripts/train_act.sh


set -e
cd /root/tfg_panda_ws

OUTPUT_DIR=/root/tfg_panda_ws/outputs/train/act_fp3_pick_place_v2

if [ -d "$OUTPUT_DIR" ]; then
    echo "[TRAIN] Borrando output_dir anterior: $OUTPUT_DIR"
    rm -rf "$OUTPUT_DIR"
fi

lerobot-train \
    --policy.type=act \
    --policy.chunk_size=30 \
    --policy.n_action_steps=30 \
    --policy.repo_id=tfg/fp3_pick_place_act \
    --dataset.repo_id=tfg/fp3_pick_place_act \
    --dataset.root=/root/tfg_panda_ws/datasets/fp3_pick_place_lerobot \
    --dataset.image_transforms.enable=true \
    --batch_size=8 \
    --steps=50000 \
    --log_freq=200 \
    --save_freq=10000 \
    --output_dir=/root/tfg_panda_ws/outputs/train/act_fp3_pick_place_v2 \
    --job_name=act_fp3_pick_place_v2

echo ""
echo "[TRAIN] === Training terminado ==="
echo "Checkpoint final: $OUTPUT_DIR/checkpoints/last/pretrained_model"
echo ""
