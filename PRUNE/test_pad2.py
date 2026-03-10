import trimesh
import numpy as np

# Create a solid box
mesh = trimesh.creation.box(extents=[10, 10, 10])

# Voxelize
voxels = mesh.voxelized(pitch=1.0).fill()
matrix = voxels.matrix

padded = np.pad(matrix, 1, mode='constant', constant_values=False)

transform = voxels.transform.copy()
transform[:3, 3] -= transform[:3, :3] @ np.array([1, 1, 1])

# Explicitly use transform kwarg
new_voxels = trimesh.voxel.VoxelGrid(padded, transform=transform)

try:
    wrap_mesh = new_voxels.marching_cubes
    print("Success! Faces:", len(wrap_mesh.faces))
    print("Original bounds:", mesh.bounds)
    print("Wrap bounds:", wrap_mesh.bounds)
except Exception as e:
    print("Failed:", e)
