#!/usr/bin/env python3
"""
Dataset Export Script for Isaac Sim

This script demonstrates how to export synthetic datasets from Isaac Sim
with annotations for perception model training.
"""

import argparse
import os
import json
import numpy as np
from pathlib import Path
import cv2
from PIL import Image


def create_coco_annotations(images_dir, output_dir, annotations_file="annotations.json"):
    """
    Create COCO format annotations for the exported dataset
    """
    # Initialize COCO format structure
    coco_format = {
        "info": {
            "description": "Synthetic Dataset from Isaac Sim",
            "version": "1.0",
            "year": 2025,
            "contributor": "Isaac Sim Synthetic Dataset Generator",
            "date_created": "2025-12-10"
        },
        "licenses": [
            {
                "id": 1,
                "name": "Synthetic Dataset License",
                "url": "http://creativecommons.org/licenses/by/4.0/"
            }
        ],
        "categories": [
            {
                "id": 1,
                "name": "robot",
                "supercategory": "object"
            },
            {
                "id": 2,
                "name": "obstacle",
                "supercategory": "object"
            },
            {
                "id": 3,
                "name": "wall",
                "supercategory": "structure"
            }
        ],
        "images": [],
        "annotations": []
    }

    # Process all images in the directory
    image_files = [f for f in os.listdir(images_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

    image_id = 1
    annotation_id = 1

    for img_file in image_files:
        img_path = os.path.join(images_dir, img_file)
        img = Image.open(img_path)
        width, height = img.size

        # Add image info
        coco_format["images"].append({
            "id": image_id,
            "file_name": img_file,
            "width": width,
            "height": height,
            "date_captured": "2025-12-10",
            "license": 1,
            "coco_url": "",
            "flickr_url": ""
        })

        # Create mock annotations (in a real scenario, these would come from Isaac Sim)
        # For this example, we'll create some simple bounding boxes
        mock_annotations = [
            {
                "id": annotation_id,
                "image_id": image_id,
                "category_id": 1,  # robot
                "bbox": [100, 100, 200, 200],  # [x, y, width, height]
                "area": 40000,
                "iscrowd": 0
            },
            {
                "id": annotation_id + 1,
                "image_id": image_id,
                "category_id": 2,  # obstacle
                "bbox": [400, 300, 150, 150],
                "area": 22500,
                "iscrowd": 0
            }
        ]

        coco_format["annotations"].extend(mock_annotations)
        annotation_id += 2
        image_id += 1

    # Save annotations to file
    annotations_path = os.path.join(output_dir, annotations_file)
    with open(annotations_path, 'w') as f:
        json.dump(coco_format, f, indent=2)

    print(f"COCO annotations saved to {annotations_path}")
    return annotations_path


def convert_depth_to_pointcloud(depth_image_path, output_path, camera_params=None):
    """
    Convert depth image to point cloud
    """
    if camera_params is None:
        # Default camera parameters
        camera_params = {
            'fx': 960.0,  # focal length x
            'fy': 540.0,  # focal length y
            'cx': 960.0,  # principal point x
            'cy': 540.0   # principal point y
        }

    # Load depth image
    depth_img = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

    if depth_img is None:
        print(f"Could not load depth image: {depth_image_path}")
        return

    height, width = depth_img.shape

    # Create coordinate grids
    x = np.arange(width)
    y = np.arange(height)
    xx, yy = np.meshgrid(x, y)

    # Convert to 3D coordinates
    zz = depth_img.astype(np.float32) / 1000.0  # Convert mm to meters
    xx = (xx - camera_params['cx']) * zz / camera_params['fx']
    yy = (yy - camera_params['cy']) * zz / camera_params['fy']

    # Reshape to point cloud format
    points = np.stack([xx.flatten(), yy.flatten(), zz.flatten()], axis=1)

    # Remove points with zero depth (invalid points)
    valid_points = points[points[:, 2] > 0]

    # Save point cloud as PLY format
    with open(output_path, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(valid_points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")

        for point in valid_points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")

    print(f"Point cloud saved to {output_path}")


def process_segmentation_masks(segmentation_dir, output_dir):
    """
    Process segmentation masks and create class mappings
    """
    # Create class mapping
    class_mapping = {
        "0": "background",
        "1": "robot",
        "2": "obstacle",
        "3": "wall",
        "4": "floor",
        "5": "ceiling"
    }

    mapping_path = os.path.join(output_dir, "class_mapping.json")
    with open(mapping_path, 'w') as f:
        json.dump(class_mapping, f, indent=2)

    print(f"Class mapping saved to {mapping_path}")
    return mapping_path


def main():
    parser = argparse.ArgumentParser(description="Export synthetic dataset from Isaac Sim")
    parser.add_argument("--input_dir", type=str, required=True,
                        help="Input directory containing Isaac Sim exported data")
    parser.add_argument("--output_dir", type=str, required=True,
                        help="Output directory for processed dataset")
    parser.add_argument("--format", type=str, default="coco",
                        choices=["coco", "kitti", "custom"],
                        help="Output format for annotations")

    args = parser.parse_args()

    # Create output directory if it doesn't exist
    Path(args.output_dir).mkdir(parents=True, exist_ok=True)

    print(f"Processing dataset from {args.input_dir}")
    print(f"Output will be saved to {args.output_dir}")

    # Process images and create COCO annotations
    if args.format == "coco":
        annotations_path = create_coco_annotations(
            images_dir=args.input_dir,
            output_dir=args.output_dir
        )

    # Process depth images to point clouds (if available)
    depth_dir = os.path.join(args.input_dir, "depth")
    if os.path.exists(depth_dir):
        depth_files = [f for f in os.listdir(depth_dir) if f.lower().endswith('.png')]
        for depth_file in depth_files:
            depth_path = os.path.join(depth_dir, depth_file)
            pc_path = os.path.join(args.output_dir, depth_file.replace('.png', '.ply'))
            convert_depth_to_pointcloud(depth_path, pc_path)

    # Process segmentation masks (if available)
    seg_dir = os.path.join(args.input_dir, "segmentation")
    if os.path.exists(seg_dir):
        mapping_path = process_segmentation_masks(seg_dir, args.output_dir)

    print("Dataset export completed successfully!")


if __name__ == "__main__":
    main()