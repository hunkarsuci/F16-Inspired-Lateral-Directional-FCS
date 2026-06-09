"""
Entry-point script for the F-16-inspired lateral-directional FCS demo.
"""

import argparse
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.simulate import run_simulation
from src.visualization import create_aircraft_animation


def parse_args():
    parser = argparse.ArgumentParser(description="Run the F-16-inspired FCS demo.")
    parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration in seconds.")
    parser.add_argument("--steps", type=int, default=5000, help="Number of simulation integration steps.")
    parser.add_argument("--out-dir", default="outputs", help="Directory for generated artifacts.")
    parser.add_argument("--fps", type=int, default=30, help="Animation frames per second.")
    parser.add_argument("--max-frames", type=int, default=240, help="Maximum frames written to the GIF.")
    parser.add_argument(
        "--aircraft-model",
        default="f16c",
        choices=["f16c", "concept"],
        help="Procedural aircraft model used in the 3D animation.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    history = run_simulation(duration=args.duration, steps=args.steps, out_dir=args.out_dir)
    animation_path = create_aircraft_animation(
        history,
        out_path=os.path.join(args.out_dir, "f16_attitude_3d.gif"),
        fps=args.fps,
        max_frames=args.max_frames,
        aircraft_model=args.aircraft_model,
    )
    print(f"SUCCESS: 3D animation saved to: {animation_path}")

if __name__ == "__main__":
    main()
