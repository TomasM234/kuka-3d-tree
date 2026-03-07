import os
import numpy as np
from dataclasses import dataclass
from enum import IntEnum

try:
    from ikpy.chain import Chain
    import yourdfpy
    import trimesh
    import pyvista as pv
except ImportError:
    Chain = None
    yourdfpy = None
    trimesh = None
    pv = None


class IKSolutionStatus(IntEnum):
    """Unified IK status codes shared by visualization and trajectory tests."""
    OK = 0
    SINGULARITY = 1
    LIMIT = 2
    UNREACHABLE = 3


@dataclass(frozen=True)
class IKEvaluation:
    """Result of evaluating an IK solution against limits/singularity checks."""
    status: IKSolutionStatus
    limit_violations: list
    singularities: list

class RobotSimulator:
    """Handles robot URDF loading, IK/FK computation, and mesh caching for 3D visualization."""

    def __init__(self):
        self.urdf_path = None
        self.ik_chain = None
        self.urdf_model = None
        self.link_meshes = {}       # link_name -> pv.PolyData
        self.active_joints = []     # list of active joint names
        self._joint_map = {}        # joint_name -> Joint object (O(1) lookup)
        self.base_elements = []

    def is_available(self):
        """Return True if ikpy and yourdfpy are installed."""
        return Chain is not None and yourdfpy is not None

    def load_robot(self, urdf_path):
        """Load a URDF robot model for IK/FK and cache its visual meshes."""
        if not self.is_available():
            raise ImportError("Required libraries for robot simulation are not installed (ikpy, yourdfpy, trimesh).")
            
        self.urdf_path = urdf_path
        
        # 1. Load for Inverse Kinematics via IKPy
        # Suppress ikpy UserWarnings about 'fixed' joints being in the active mask by default
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            self.ik_chain = Chain.from_urdf_file(urdf_path)
            
        # Drop strict physical bounds from the IK solver. 
        # We need the math model to track the target continuously so we can visually flag off-limit axes in red.
        for link in self.ik_chain.links:
            link.bounds = (-np.inf, np.inf)
        
        # 2. Load for Forward Kinematics and Visualization via yourdfpy
        self.urdf_model = yourdfpy.URDF.load(urdf_path)
        
        self.active_joints = [joint.name for joint in self.urdf_model.robot.joints if joint.type != 'fixed']
        
        # Build O(1) lookup map for joint limits
        self._joint_map = {j.name: j for j in self.urdf_model.robot.joints}
        
        # Cache meshes
        self._cache_meshes(os.path.dirname(urdf_path))
        
    def _cache_meshes(self, base_dir):
        """Preload all STL/DAE meshes from the URDF to avoid reloading them."""
        self.link_meshes.clear()
        
        for link in self.urdf_model.robot.links:
            # A link might have multiple visuals, we combine them into one trimesh or keep as list
            # For simplicity, let's extract the main visual geometry if present
            # yourdfpy often uses trimesh internally
            meshes = []
            for visual in link.visuals:
                if visual.geometry.mesh is not None:
                    mesh_filename = visual.geometry.mesh.filename
                    if not mesh_filename:
                        continue
                        
                    # Handle package:// or relative paths
                    if mesh_filename.startswith('package://'):
                        # typical ros notation, strip and join
                        # We assume the user has the meshes folder near the urdf or one level up
                        rel_path = mesh_filename.replace('package://', '')
                        # Hack to find the right path: split rel_path
                        parts = rel_path.split('/')
                        if len(parts) > 1:
                            probe_path = os.path.join(base_dir, '..', *parts[1:])
                            if not os.path.exists(probe_path):
                                probe_path = os.path.join(base_dir, *parts)
                            if os.path.exists(probe_path):
                                mesh_filename = probe_path
                                
                    if not os.path.isabs(mesh_filename):
                        mesh_filename = os.path.normpath(os.path.join(base_dir, mesh_filename))
                        
                    if os.path.exists(mesh_filename):
                        try:
                            m = trimesh.load_mesh(mesh_filename)
                            # apply visual origin transform if any
                            if visual.origin is not None:
                                m.apply_transform(visual.origin)
                            meshes.append(m)
                        except Exception as e:
                            print(f"Failed to load mesh {mesh_filename}: {e}")
                            
            if meshes:
                if len(meshes) == 1:
                    m = meshes[0]
                else:
                    m = trimesh.util.concatenate(meshes)
                    
                # Convert trimesh to PyVista PolyData
                faces = np.column_stack((np.full(len(m.faces), 3), m.faces)).flatten()
                pv_mesh = pv.PolyData(m.vertices, faces)
                # Scale from URDF meters to PyVista mm
                pv_mesh.points *= 1000.0
                self.link_meshes[link.name] = pv_mesh

    def build_seed_from_active_angles(self, active_joint_angles):
        """Map active-joint angles to the full IK chain array."""
        if self.ik_chain is None:
            return None

        seed = np.zeros(len(self.ik_chain.links), dtype=np.float64)
        angle_idx = 0
        for i, link in enumerate(self.ik_chain.links):
            if link.name in self.active_joints and angle_idx < len(active_joint_angles):
                seed[i] = active_joint_angles[angle_idx]
                angle_idx += 1
        return seed

    def apply_base_azimuth_to_seed(self, seed_template, target_position_m):
        """Adjust A1 seed toward target azimuth while preserving front/back branch."""
        if self.ik_chain is None:
            return np.array(seed_template, dtype=np.float64, copy=True)

        dyn = np.array(seed_template, dtype=np.float64, copy=True)
        base_azimuth = np.arctan2(target_position_m[1], target_position_m[0])
        is_back = False

        for i, link in enumerate(self.ik_chain.links):
            if link.name in self.active_joints:
                if dyn[i] > 1.0:
                    is_back = True
                dyn[i] = base_azimuth + (np.pi if is_back else 0.0)
                break

        return dyn

    @staticmethod
    def _distance_to_interval(value, lower, upper):
        if value < lower:
            return lower - value
        if value > upper:
            return value - upper
        return 0.0

    def _wrap_joint_value(self, value, lower, upper, reference=None):
        """Pick best 2*pi-equivalent value: in-limit if possible, else closest to limits."""
        candidates = [value + 2.0 * np.pi * k for k in range(-3, 4)]
        in_limit = [c for c in candidates if lower <= c <= upper]

        if in_limit:
            if reference is not None:
                return min(in_limit, key=lambda c: abs(c - reference))
            return min(in_limit, key=lambda c: abs(c - value))

        if reference is not None:
            return min(candidates, key=lambda c: (self._distance_to_interval(c, lower, upper), abs(c - reference)))
        return min(candidates, key=lambda c: self._distance_to_interval(c, lower, upper))

    def normalize_solution_to_limits(self, joint_angles, reference=None):
        """Normalize each active joint by 2*pi wrapping toward URDF limits."""
        if joint_angles is None:
            return None
        if self.urdf_model is None or self.ik_chain is None:
            return joint_angles

        normalized = np.array(joint_angles, dtype=np.float64, copy=True)
        reference_arr = None if reference is None else np.asarray(reference, dtype=np.float64)

        for i, link in enumerate(self.ik_chain.links):
            if link.name not in self.active_joints:
                continue
            j = self._joint_map.get(link.name)
            if not j or j.limit is None:
                continue

            lower = j.limit.lower if j.limit.lower is not None else -float('inf')
            upper = j.limit.upper if j.limit.upper is not None else float('inf')
            ref_val = None
            if reference_arr is not None and i < len(reference_arr):
                ref_val = reference_arr[i]

            normalized[i] = self._wrap_joint_value(normalized[i], lower, upper, ref_val)

        return normalized

    def evaluate_solution(self, joint_angles, singularity_threshold_deg=1.0):
        """Evaluate a solution and return unified status + detail lists."""
        if joint_angles is None:
            return IKEvaluation(IKSolutionStatus.UNREACHABLE, [], [])

        limit_violations = self.check_limits(joint_angles)
        singularities = self.check_singularities(joint_angles, threshold_deg=singularity_threshold_deg)

        if limit_violations:
            status = IKSolutionStatus.LIMIT
        elif singularities:
            status = IKSolutionStatus.SINGULARITY
        else:
            status = IKSolutionStatus.OK

        return IKEvaluation(status, limit_violations, singularities)


    def calculate_ik(self, target_position, target_orientation=None, initial_position=None):
        """Calculate Inverse Kinematics for a given XYZ target (in meters).

        Args:
            target_position: [x, y, z] target in meters.
            target_orientation: Optional 3×3 rotation matrix for the end-effector.
            initial_position: Optional joint-angle array to seed the optimizer.
                              Using the previous IK solution dramatically speeds up
                              convergence for sequential trajectory points.
        Returns:
            Array of joint angles, or None if no chain is loaded.
        """
        if not self.ik_chain:
            return None

        kwargs = {}
        if initial_position is not None:
            kwargs['initial_position'] = initial_position

        if target_orientation is not None:
            ik_solution = self.ik_chain.inverse_kinematics(
                target_position, target_orientation,
                orientation_mode="all", **kwargs)
        else:
            ik_solution = self.ik_chain.inverse_kinematics(
                target_position, **kwargs)

        if ik_solution is None:
            return None

        return self.normalize_solution_to_limits(ik_solution, reference=initial_position)
        
    def get_forward_transforms(self, joint_angles):
        """
        Returns a dictionary of link_name -> 4x4 transformation matrix
        joint_angles: List of angles matching the active joints in URDF.
        Note: ik_solution includes the base fixed joint.
        """
        if not self.urdf_model:
            return {}
            
        # ik_solution usually has length len(ik_chain.links) which includes base and dummy links.
        # We need to feed the true active joint values to yourdfpy dictionary
        
        cfg = {}
        # match ikpy links to yourdfpy joints.
        # ikpy output is an array corresponding to its internal links. 
        # Active joints in ikpy have the same names.
        for i, link in enumerate(self.ik_chain.links):
            if link.name in self.active_joints:
                cfg[link.name] = joint_angles[i]
                
        self.urdf_model.update_cfg(cfg)
        
        # Extract forward kinematics transforms for all links
        transforms = {}
        for link_name in self.link_meshes.keys():
            t_matrix = np.copy(self.urdf_model.scene.graph.get(link_name)[0])
            # Scale the translation part from meters to mm to match PyVista scene
            t_matrix[0:3, 3] *= 1000.0
            transforms[link_name] = t_matrix
            
        return transforms

    def check_limits(self, joint_angles):
        """Return a list of child link names whose joints exceed their URDF limits."""
        if not self.urdf_model:
            return []

        violations = []
        for i, link in enumerate(self.ik_chain.links):
            if link.name in self.active_joints:
                angle = joint_angles[i]
                j = self._joint_map.get(link.name)
                if j and j.limit is not None:
                    lower = j.limit.lower if j.limit.lower is not None else -float('inf')
                    upper = j.limit.upper if j.limit.upper is not None else float('inf')
                    if angle < lower or angle > upper:
                        violations.append(j.child)
        return violations

    def check_singularities(self, joint_angles, threshold_deg=1.0):
        """Detect wrist singularity (joint 5 near zero). Returns list of affected child link names."""
        if not self.urdf_model or not self.ik_chain:
            return []
            
        singularities = []
        active_links = [l for l in self.ik_chain.links if l.name in self.active_joints]
        
        # Standard industrial 6-DOF robot: joint 5 is index 4 (0-based)
        if len(active_links) >= 6:
            j4 = active_links[3]
            j5 = active_links[4]
            j6 = active_links[5]
            
            j5_idx = self.ik_chain.links.index(j5)
            angle_5 = joint_angles[j5_idx]
            
            # Wrist singularity: joint 5 angle is close to 0 degrees
            if abs(np.degrees(angle_5)) <= threshold_deg:
                for name in [j4.name, j6.name]:
                    j = self._joint_map.get(name)
                    if j:
                        singularities.append(j.child)
                        
        return singularities
