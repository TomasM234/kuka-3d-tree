import trimesh
import numpy as np

# Create a solid box (this will cause the marching cubes error if not padded)
mesh = trimesh.creation.box(extents=[10, 10, 10])

# Voxelize
voxels = mesh.voxelized(pitch=1.0).fill()
matrix = voxels.matrix

print("Original shape:", matrix.shape)
print("Original transform:")
print(voxels.transform)

# Pad to prevent the ValueError
padded = np.pad(matrix, 1, mode='constant', constant_values=False)

# Adjust transform
transform = voxels.transform.copy()
# Voxel coordinates shift by -1, -1, -1
# World shift = Transform * [-1, -1, -1, 1]^T
new_origin = transform @ np.array([-1, -1, -1, 1])
transform[:3, 3] = new_origin[:3]

print("New shape:", padded.shape)
print("New transform:")
print(transform)

new_voxels = trimesh.voxel.VoxelGrid(padded, transform)

try:
    wrap_mesh = new_voxels.marching_cubes
    print("Success! Faces:", len(wrap_mesh.faces))
    
    # Check bounds to ensure they closely match original
    print("Original bounds:", mesh.bounds)
    print("Wrap bounds:", wrap_mesh.bounds)
except Exception as e:
    print("Failed:", e)
