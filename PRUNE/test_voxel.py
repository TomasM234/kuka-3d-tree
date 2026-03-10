import trimesh
import numpy as np
import time

# Create a complex shape
mesh = trimesh.creation.torus(major_radius=10, minor_radius=2)
print("Original mesh faces:", len(mesh.faces))

# We want to wrap this tightly but close small holes/intersections.
# Voxelization method
t0 = time.time()
pitch = 0.5 # voxel size
voxels = mesh.voxelized(pitch=pitch)

# We can fill the voxels (this makes it solid inside)
solid_voxels = voxels.fill()

# Convert back to mesh via marching cubes
wrapped_mesh = solid_voxels.as_boxes() # Or marching_cubes
print("Voxel -> Box Wrap Mesh faces:", len(wrapped_mesh.faces), "- Time:", time.time() - t0)

t1 = time.time()
import scipy.ndimage as nd
# Use morphological closing on the solid voxel matrix
matrix = solid_voxels.matrix
# Dilate and erode to close holes larger than the elements
struct = nd.generate_binary_structure(3, 3) # 3D sphere-like structure
dilated = nd.binary_dilation(matrix, structure=struct, iterations=3)
eroded = nd.binary_erosion(dilated, structure=struct, iterations=3)

# Create new voxel grid
new_voxels = trimesh.voxel.VoxelGrid(eroded, solid_voxels.transform)
smooth_wrap = new_voxels.marching_cubes
print("Morphological Wrap Mesh faces:", len(smooth_wrap.faces), "- Time:", time.time() - t1)

smooth_wrap.export("test_wrap.stl")
print("Done")
