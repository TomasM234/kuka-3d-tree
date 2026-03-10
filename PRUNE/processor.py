import logging
from pathlib import Path

import pyvista as pv
import pyvista as pv
import trimesh
import numpy as np

logger = logging.getLogger(__name__)

def load_mesh(file_path: str) -> pv.PolyData:
    """Loads a CAD/Mesh file and returns a pyvista PolyData object.
    Supports STL, OBJ natively through pyvista.
    Supports STEP/STP through gmsh.
    """
    path = Path(file_path)
    ext = path.suffix.lower()
    
    if ext in ['.stl', '.obj']:
        # Load natively using pyvista
        mesh = pv.read(str(path))
        if not isinstance(mesh, pv.PolyData):
            # Convert UnstructuredGrid or others to PolyData
            mesh = mesh.extract_surface()
        return mesh
    
    elif ext in ['.stp', '.step']:
        return _load_step_via_gmsh(str(path))
    
    else:
        raise ValueError(f"Unsupported file format: {ext}")

def save_stl(mesh: pv.PolyData, file_path: str):
    """Saves the PyVista mesh as an STL file."""
    mesh.save(file_path)

def apply_decimation(mesh: pv.PolyData, target_reduction: float) -> pv.PolyData:
    """
    Reduces the number of triangles in the mesh by target_reduction.
    E.g. target_reduction=0.9 removes 90% of triangles.
    """
    # PyVista decimate directly reduces the mesh geometry
    # Check if the mesh is pure triangles, decimate only works on triangles
    tri_mesh = mesh.triangulate()
    decimated = tri_mesh.decimate(target_reduction)
    return decimated

def apply_convex_hull(mesh: pv.PolyData) -> pv.PolyData:
    """
    Generates a convex hull around the provided mesh using trimesh,
    then converts it back to pyvista format.
    """
    # We use trimesh for the convex hull computation as it's very robust
    tm_mesh = trimesh.Trimesh(vertices=mesh.points, faces=mesh.faces.reshape(-1, 4)[:, 1:])
    hull_tm = tm_mesh.convex_hull
    
    # Convert back to pyvista
    n_faces = len(hull_tm.faces)
    # Pyvista needs faces formatted as [n_points_in_face, p1, p2, p3, ...]
    padding = np.full((n_faces, 1), 3, dtype=np.int64)
    pv_faces = np.hstack((padding, hull_tm.faces)).flatten()
    
    hull_pv = pv.PolyData(hull_tm.vertices, pv_faces)
    return hull_pv

def apply_voxel_shrinkwrap(mesh: pv.PolyData, pitch: float) -> pv.PolyData:
    """
    Wraps the mesh tightly while preserving large concavities. 
    It voxelizes the mesh, closes small holes via morphology, and extracts the surface.
    """
    # 1. Convert to trimesh
    tm_mesh = trimesh.Trimesh(vertices=mesh.points, faces=mesh.faces.reshape(-1, 4)[:, 1:])
    
    # 2. Voxelize
    voxels = tm_mesh.voxelized(pitch=pitch)
    solid_voxels = voxels.fill()
    
    # 3. Morphological closing to seal small tears
    import scipy.ndimage as nd
    matrix = solid_voxels.matrix
    
    # Pad matrix to ensure the mesh has a closed surface and doesn't crash on solid chunks
    padded = np.pad(matrix, 1, mode='constant', constant_values=False)
    
    struct = nd.generate_binary_structure(3, 3) 
    dilated = nd.binary_dilation(padded, structure=struct, iterations=2)
    eroded = nd.binary_erosion(dilated, structure=struct, iterations=2)
    
    # 4. Extract surface via marching cubes
    import skimage.measure as measure
    v, f, n, _ = measure.marching_cubes(eroded, level=0.5)
    
    # Translate vertices from padded voxel indices back to world coordinates
    v = (v - 1.0) * pitch + solid_voxels.transform[:3, 3]
    
    # 5. Convert back to PyVista
    n_faces = len(f)
    padding_arr = np.full((n_faces, 1), 3, dtype=np.int64)
    pv_faces = np.hstack((padding_arr, f)).flatten()
    
    return pv.PolyData(v, pv_faces)

def _load_step_via_gmsh(file_path: str) -> pv.PolyData:
    """
    Loads a STEP file using the gmsh python API,
    fragments it into a 2D surface mesh, and extracts via PyVista.
    import gmsh is deferred so it doesn't fail if gmsh is not installed yet.
    """
    import gmsh
    
    gmsh.initialize()
    # Don't print output to terminal
    gmsh.option.setNumber("General.Terminal", 0) 
    
    try:
        # Load the STEP file
        gmsh.merge(file_path)
        
        # We need a 2D mesh on the surfaces
        # This will create a mesh of triangles
        gmsh.option.setNumber("Mesh.MeshSizeFactor", 0.5) # Adjust density here if needed
        gmsh.model.mesh.generate(2) # 2D mesh
        
        # Extract node coordinates
        node_tags, node_coords, _ = gmsh.model.mesh.getNodes()
        # gmsh returns 1-indexed node tags, we want an array where index 0 is node tag 1
        # It's safer to build a mapping dictionary or array
        coords = np.array(node_coords).reshape(-1, 3)
        tag_to_idx = {tag: i for i, tag in enumerate(node_tags)}
        
        # Extract element connectivity (triangles only for surface)
        # element_types, element_tags, node_tags_by_element
        element_types, element_tags, node_tags_per_el = gmsh.model.mesh.getElements()
        
        faces = []
        for i, el_type in enumerate(element_types):
            if el_type == 2: # 2 is the type for 3-node triangles in gmsh
                nodes = node_tags_per_el[i]
                for j in range(0, len(nodes), 3):
                    # convert gmsh node tags to our numpy array indices
                    n1 = tag_to_idx[nodes[j]]
                    n2 = tag_to_idx[nodes[j+1]]
                    n3 = tag_to_idx[nodes[j+2]]
                    faces.extend([3, n1, n2, n3])
        
        if not faces:
            raise ValueError("No triangle surface mesh was generated by GMSH from the STEP file.")
            
        mesh_pv = pv.PolyData(coords, np.array(faces))
        return mesh_pv
        
    finally:
        gmsh.finalize()
