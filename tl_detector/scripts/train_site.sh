#!/bin/bash
cd ..
python train.py --logtostderr --train_dir=./models/train_site --pipeline_config_path=faster_rcnn_inception_v2_coco_site.config
