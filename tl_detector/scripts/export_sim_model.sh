#!/bin/bash
cd ..
python export_inference_graph.py --input_type=image_tensor --pipeline_config_path=faster_rcnn_inception_v2_coco_sim.config --trained_checkpoint_prefix=./models/train/model.ckpt-25000 --output_directory=./fine_tuned_sim_model
