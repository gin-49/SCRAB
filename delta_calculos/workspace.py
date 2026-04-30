#!/usr/bin/env python3
"""
Delta Robot XZ Plane Workspace Analyzer - WITH EXISTING DELTAKINEMATICS CLASS
This script integrates with your existing DeltaKinematics import and adds workspace visualization.
FIXED: Properly handles file paths and creates directories automatically.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
import warnings
import os
from pathlib import Path

warnings.filterwarnings("ignore")

# IMPORTANT: Make sure DeltaKinematics is in your Python path
try:
    from DeltaKinematics import DeltaKinematics

    print("✓ Successfully imported DeltaKinematics")
except ImportError as e:
    print(f"✗ Error importing DeltaKinematics: {e}")
    print("Make sure DeltaKinematics.py is in your Python path")
    exit(1)


class DeltaWorkspaceAnalyzerWithDK:
    """
    Workspace analyzer using your existing DeltaKinematics class.
    """

    def __init__(self, rf, re, f, e):
        """
        Initialize with your DeltaKinematics instance.

        Args:
            rf: Arm length (bicep) in meters
            re: Forearm length (brazo) in meters
            f: Base triangle side length in meters
            e: End-effector platform side length in meters
        """
        self.rf = rf
        self.re = re
        self.f = f
        self.e = e

        # Create instance of your DeltaKinematics class
        self.delta = DeltaKinematics(rf, re, f, e)

        print(f"Delta Robot Parameters:")
        print(f"  Arm length (rf):        {self.rf * 1000:.1f} mm")
        print(f"  Forearm length (re):    {self.re * 1000:.1f} mm")
        print(f"  Base triangle (f):      {self.f * 1000:.1f} mm")
        print(f"  Platform triangle (e):  {self.e * 1000:.1f} mm")
        print()

    def is_reachable(self, x, y, z):
        """
        Check if a position is reachable using your DeltaKinematics.ik() method.

        DeltaKinematics.ik() returns:
          - numpy array of 3 angles  → reachable
          - -1 (int)                 → out of reach
          - array([nan, nan, nan])   → geometrically invalid (e.g. z=0)

        Args:
            x, y, z: Position in meters

        Returns:
            Tuple (is_reachable, angles) where angles is from ik() or None
        """
        try:
            angles = self.delta.ik([x, y, z])

            # Case 1: ik() returned -1 (out of reach)
            if isinstance(angles, int):
                return False, None

            # Case 2: ik() returned nan values (geometrically invalid)
            if np.any(np.isnan(angles)):
                return False, None

            # Case 3: valid angles array
            return True, angles

        except Exception:
            return False, None

    def scan_xz_plane(self, x_min, x_max, z_min, z_max, x_step, z_step, y=0):
        """
        Scan the XZ plane for reachable positions.

        Args:
            x_min, x_max: X range in meters
            z_min, z_max: Z range in meters
            x_step, z_step: Step size in meters
            y: Y coordinate (constant) in meters

        Returns:
            Dictionary with reachable and unreachable points
        """
        reachable = []
        unreachable = []

        x_range = np.arange(x_min, x_max + x_step, x_step)
        z_range = np.arange(z_min, z_max + z_step, z_step)

        total_points = len(x_range) * len(z_range)
        current = 0

        print(f"Scanning {total_points} points in XZ plane (y={y * 1000:.1f}mm)...")
        print(f"X range: {x_min * 1000:.1f} to {x_max * 1000:.1f} mm")
        print(f"Z range: {z_min * 1000:.1f} to {z_max * 1000:.1f} mm")
        print(f"Resolution: {x_step * 1000:.1f}mm x {z_step * 1000:.1f}mm")
        print()

        for x in x_range:
            for z in z_range:
                current += 1
                if current % max(1, total_points // 10) == 0:
                    progress = (current / total_points) * 100
                    print(f"  Progress: {progress:.0f}%", end="\r")

                is_reach, angles = self.is_reachable(x, y, z)

                if is_reach:
                    reachable.append([x, z])
                else:
                    unreachable.append([x, z])

        print(f"  Progress: 100%")
        print(f"Reachable: {len(reachable)} points")
        print(f"Unreachable: {len(unreachable)} points")
        print()

        return {
            "reachable": np.array(reachable) if reachable else np.array([]),
            "unreachable": np.array(unreachable) if unreachable else np.array([]),
        }

    def plot_workspace(
        self,
        scan_results,
        title="Delta Robot XZ Plane Workspace",
        figsize=(14, 10),
        show_unreachable=False,
        save_path=None,
    ):
        """
        Plot the workspace visualization.

        Args:
            scan_results: Dictionary from scan_xz_plane()
            title: Plot title
            figsize: Figure size tuple
            show_unreachable: Whether to show unreachable points
            save_path: If provided, save plot to this path
        """
        fig, ax = plt.subplots(figsize=figsize)

        reachable = scan_results["reachable"]
        unreachable = scan_results["unreachable"]

        # Plot unreachable points (optional)
        if show_unreachable and len(unreachable) > 0:
            ax.scatter(
                unreachable[:, 0] * 1000,
                unreachable[:, 1] * 1000,
                c="lightcoral",
                s=1,
                alpha=0.05,
                label="Unreachable",
            )

        # Plot reachable points
        if len(reachable) > 0:
            ax.scatter(
                reachable[:, 0] * 1000,
                reachable[:, 1] * 1000,
                c="lightblue",
                s=8,
                alpha=0.7,
                label="Reachable Points",
            )

            # Create convex hull and fill workspace
            try:
                if len(reachable) > 3:
                    hull = ConvexHull(reachable)
                    hull_points = reachable[hull.vertices]

                    # Create filled polygon
                    polygon = Polygon(
                        hull_points * 1000,
                        alpha=0.25,
                        facecolor="cornflowerblue",
                        edgecolor="darkblue",
                        linewidth=3,
                    )
                    ax.add_patch(polygon)

                    # Print workspace statistics
                    x_min, x_max = (
                        reachable[:, 0].min() * 1000,
                        reachable[:, 0].max() * 1000,
                    )
                    z_min, z_max = (
                        reachable[:, 1].min() * 1000,
                        reachable[:, 1].max() * 1000,
                    )
                    print("Workspace Statistics:")
                    print(
                        f"  X range: {x_min:.1f} to {x_max:.1f} mm (span: {x_max - x_min:.1f}mm)"
                    )
                    print(
                        f"  Z range: {z_min:.1f} to {z_max:.1f} mm (span: {z_max - z_min:.1f}mm)"
                    )
                    area_mm2 = hull.volume * 1e6  # Convert m² to mm²
                    print(f"  Workspace area: {area_mm2:.1f} mm²")
                    print()
            except Exception as e:
                print(f"Note: Could not create convex hull: {e}")

        # Formatting
        ax.set_xlabel("X Position (mm)", fontsize=13, fontweight="bold")
        ax.set_ylabel("Z Position (mm)", fontsize=13, fontweight="bold")
        ax.set_title(title, fontsize=14, fontweight="bold", pad=20)
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.set_aspect("equal")
        ax.legend(fontsize=11, loc="upper right")

        # Add parameter info box
        param_text = (
            f"Robot Parameters:\n"
            f"Arm (rf):       {self.rf * 1000:.0f} mm\n"
            f"Forearm (re):   {self.re * 1000:.0f} mm\n"
            f"Base (f):       {self.f * 1000:.0f} mm\n"
            f"Platform (e):   {self.e * 1000:.0f} mm\n"
            f"\n"
            f"Points Tested:  {len(reachable) + len(unreachable)}\n"
            f"Reachable:      {len(reachable)}"
        )

        ax.text(
            0.02,
            0.98,
            param_text,
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment="top",
            family="monospace",
            bbox=dict(
                boxstyle="round",
                facecolor="lightyellow",
                alpha=0.9,
                edgecolor="gray",
                linewidth=1.5,
            ),
        )

        plt.tight_layout()

        if save_path:
            # Expand user path (~) and create directories if needed
            save_path = Path(save_path).expanduser()
            save_path.parent.mkdir(parents=True, exist_ok=True)

            plt.savefig(str(save_path), dpi=150, bbox_inches="tight")
            print(f"✓ Plot saved to: {save_path}")

        return fig, ax

    def test_point(self, x, y, z):
        """
        Test a single point and print detailed results.

        Args:
            x, y, z: Position in meters
        """
        print(
            f"\nTesting point: X={x * 1000:.2f}mm, Y={y * 1000:.2f}mm, Z={z * 1000:.2f}mm"
        )

        is_reach, angles = self.is_reachable(x, y, z)

        if is_reach:
            print(f"✓ REACHABLE")
            if angles is not None:
                # Your ik() method returns angle values
                if isinstance(angles, (list, tuple, np.ndarray)):
                    print(f"  Joint Angles: {angles}")
                else:
                    print(f"  Result: {angles}")
        else:
            print(f"✗ UNREACHABLE")


def main():
    """Main execution with your DeltaKinematics class."""

    print("=" * 75)
    print("DELTA ROBOT XZ PLANE WORKSPACE ANALYZER")
    print("Using your DeltaKinematics class")
    print("=" * 75)
    print()

    # Your robot parameters (from your code)
    rf = 250 / 1000  # bicep (arm) - 250mm
    re = 500 / 1000  # brazo (forearm) - 500mm
    f = 300 / 1000  # base triangle - 300mm
    e = 25 / 1000  # end effector platform - 25mm

    # Create analyzer with your parameters
    analyzer = DeltaWorkspaceAnalyzerWithDK(rf, re, f, e)

    # ========== TEST SINGLE POINT ==========
    print("SINGLE POINT TEST:")
    print("-" * 75)
    analyzer.test_point(x=-9 / 1000, y=14 / 1000, z=-315 / 1000)

    # ========== CUSTOMIZE SCAN PARAMETERS HERE ==========
    print("\n" + "=" * 75)
    print("WORKSPACE SCAN CONFIGURATION:")
    print("=" * 75)
    print()

    # MODIFY THESE VALUES FOR YOUR ANALYSIS
    # -------- X LIMITS (mm to meters) --------
    x_min = -300 / 1000  # -150mm
    x_max = 300 / 1000  # +150mm

    # -------- Z LIMITS (mm to meters) --------
    z_min = -600 / 1000  # -600mm
    z_max = 0 / 1000  # -200mm

    # -------- RESOLUTION (smaller = more detailed but slower) --------
    x_step = 2 / 1000  # 5mm step
    z_step = 2 / 1000  # 5mm step
    # For faster scans, try: 10/1000 (10mm)
    # For more detail, try: 2/1000 (2mm) or 1/1000 (1mm)

    # -------- Y POSITION (XZ plane at this Y value) --------
    y = 0 / 1000  # Center plane (0mm)

    print(f"Scanning limits:")
    print(f"  X: {x_min * 1000:.0f} to {x_max * 1000:.0f} mm")
    print(f"  Z: {z_min * 1000:.0f} to {z_max * 1000:.0f} mm")
    print(f"  Y: {y * 1000:.0f} mm (constant)")
    print(f"Resolution: {x_step * 1000:.1f}mm × {z_step * 1000:.1f}mm")
    print()

    # Perform scan
    print("=" * 75)
    results = analyzer.scan_xz_plane(
        x_min=x_min,
        x_max=x_max,
        z_min=z_min,
        z_max=z_max,
        x_step=x_step,
        z_step=z_step,
        y=y,
    )

    # Generate plot
    print("=" * 75)
    print("GENERATING VISUALIZATION...")
    print("=" * 75)
    print()

    # CHANGE THIS PATH TO WHERE YOU WANT TO SAVE
    # Options:
    # - "./delta_workspace.png" (current directory)
    # - "~/SCRAB/delta_calculos/delta_workspace.png" (home directory)
    # - "/absolute/path/to/delta_workspace.png"

    output_path = "~/SCRAB/delta_calculos/test/delta_workspace.png"

    fig, ax = analyzer.plot_workspace(
        results,
        title=(
            f"Delta Robot XZ Plane Workspace\n"
            f"(rf={rf * 1000:.0f}mm, re={re * 1000:.0f}mm, "
            f"f={f * 1000:.0f}mm, e={e * 1000:.0f}mm, y={y * 1000:.0f}mm)"
        ),
        figsize=(14, 10),
        show_unreachable=False,  # Set True to see unreachable points
        save_path=output_path,
    )

    plt.show()

    print("\n" + "=" * 75)
    print("HOW TO CUSTOMIZE:")
    print("=" * 75)
    print("""
1. CHANGE ROBOT GEOMETRY:
   Edit these lines in main():
       rf = 250 / 1000  # Change arm length
       re = 500 / 1000  # Change forearm length
       f = 300 / 1000   # Change base triangle
       e = 25 / 1000    # Change platform triangle

2. CHANGE SCAN AREA:
   Modify X and Z limits (in meters):
       x_min = -150 / 1000  # Left limit
       x_max = 150 / 1000   # Right limit
       z_min = -600 / 1000  # Bottom limit
       z_max = -200 / 1000  # Top limit

3. CHANGE RESOLUTION:
   Adjust step size (smaller = more detail but slower):
       x_step = 5 / 1000    # 5mm steps (try 10, 2, or 1)
       z_step = 5 / 1000    # 5mm steps

   Recommendation:
   - Fast: 10/1000 (10mm) - good for testing
   - Good: 5/1000 (5mm) - recommended balance ⭐
   - Detailed: 2/1000 (2mm) - takes longer
   - Very detailed: 1/1000 (1mm) - much slower

4. TEST DIFFERENT Y PLANES:
   Change y parameter to scan at different XY planes:
       y = 0 / 1000     # Center (y=0)
       y = 50 / 1000    # Offset 50mm
       y = -50 / 1000   # Offset -50mm

5. CHANGE OUTPUT PATH:
   Modify output_path variable:
       output_path = "./delta_workspace.png"  # Current dir
       output_path = "~/SCRAB/delta_calculos/test/delta_workspace.png"  # Home dir
       output_path = "/tmp/delta_workspace.png"  # Temp dir

6. SHOW UNREACHABLE POINTS:
   Set show_unreachable=True in plot_workspace() to visualize
   unreachable areas as well (slower visualization).

NOTES:
- Paths with ~ (tilde) are automatically expanded to your home directory
- Directories are created automatically if they don't exist
- Higher resolution scans take longer but show more detail
    """)


if __name__ == "__main__":
    main()
