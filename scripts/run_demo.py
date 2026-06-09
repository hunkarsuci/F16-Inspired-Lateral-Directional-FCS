"""
Entry-point script for the F-16-inspired lateral-directional FCS demo.
"""

import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.simulate import run_simulation
from src.visualization import create_aircraft_animation

def main():
    history = run_simulation()
    animation_path = create_aircraft_animation(history)
    print(f"SUCCESS: 3D animation saved to: {animation_path}")

if __name__ == "__main__":
    main()
