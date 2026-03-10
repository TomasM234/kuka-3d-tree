import pyvista as pv
import numpy as np

# Create a donut-like shape with gaps (a torus)
mesh = pv.ParametricTorus().triangulate()
points = mesh.points

# Test Delaunay 3D with Alpha
print("Original points:", len(points))
try:
    # Alpha shape
    # alpha=0.5 (example). Smaller = tighter, larger = more convex
    vol = pv.PolyData(points).delaunay_3d(alpha=0.5)
    surf = vol.extract_surface()
    print("Alpha Shape surface cells:", surf.n_cells)
    print("Success")
except Exception as e:
    print("Error:", e)
