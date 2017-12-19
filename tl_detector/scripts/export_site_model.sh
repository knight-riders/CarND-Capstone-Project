#!/bin/bash
cd ..
python export_inference_graph.py --input_type=image_tensor --pipeline_config_path=faster_rcnn_inception_v2_coco_site.config --trained_checkpoint_prefix=./models/train_site/model.ckpt-6000 --output_directory=./fine_tuned_site_model
