#!/usr/bin/env python3
"""
Semantic Mapping Pipeline — SAM → CLIP → Graph

Usage:
  # 1. Run SLAM first to get poses:
  #    ./run_slam_ntuviral.sh /mnt/data/eee_01.bag /mnt/data/semantic_output

  # 2. Build semantic map:
  python run_pipeline.py \
    --bag    /mnt/data/eee_01.bag \
    --lidar  /os_cloud_node/points \
    --camera /cam0/image_raw \
    --poses  /mnt/data/semantic_output/poses_tum.txt \
    --output /mnt/data/semantic_output

  # 3. Query the map:
  python run_pipeline.py --query "red bench in garden"
"""
import argparse
from semantic_mapping import SemanticMappingPipeline, Config, query_graph


def main():
    p = argparse.ArgumentParser()
    # Pipeline args
    p.add_argument("--bag",        default="/mnt/data/eee_01.bag")
    p.add_argument("--lidar",      default="/os_cloud_node/points")
    p.add_argument("--camera",     default="/cam0/image_raw")
    p.add_argument("--poses",      default="")
    p.add_argument("--output",     default="/mnt/data/semantic_output")
    p.add_argument("--every-n",    type=int,   default=10)
    p.add_argument("--max-frames", type=int,   default=300)
    p.add_argument("--sam",        default="/mnt/data/sam_vit_h_4b8939.pth")
    p.add_argument("--sam-type",   default="vit_h")
    p.add_argument("--blip",       action="store_true",
                   help="Enable BLIP-2 captions (needs ~12GB VRAM)")
    # Query args
    p.add_argument("--query",  default="")
    p.add_argument("--graph",  default="/mnt/data/semantic_output/semantic_graph.pkl")
    args = p.parse_args()

    if args.query:
        pos = query_graph(args.graph, args.query)
        if pos is not None:
            print(f"\n✓ '{args.query}'  →  position {pos}")
        else:
            print(f"\n✗ '{args.query}'  not found in graph")
        return

    cfg = Config(
        bag_path        = args.bag,
        lidar_topic     = args.lidar,
        camera_topic    = args.camera,
        poses_file      = args.poses,
        output_dir      = args.output,
        lidar_every_n   = args.every_n,
        max_frames      = args.max_frames,
        sam_checkpoint  = args.sam,
        sam_model_type  = args.sam_type,
        use_blip        = args.blip,
        graph_file      = f"{args.output}/semantic_graph.pkl",
    )

    pipeline = SemanticMappingPipeline(cfg)
    graph = pipeline.run()

    print("\n" + "=" * 60)
    print(f"Graph: {graph.stats()}")
    print(f"\nTo query:")
    print(f"  python run_pipeline.py --query 'red bus in garden' --graph {cfg.graph_file}")
    print("=" * 60)


if __name__ == "__main__":
    main()
