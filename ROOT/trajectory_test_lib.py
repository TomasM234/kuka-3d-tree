import multiprocessing
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
import numpy as np

from pose_math import kuka_base_to_matrix


@dataclass(frozen=True)
class TrajectoryTestConfig:
    """Configuration shared by all trajectory-test chunks."""
    urdf_path: str
    base_params: tuple
    tool_params: tuple
    seed_templates: tuple
    max_workers: int | None = None
    chunk_size: int | None = None


@dataclass(frozen=True)
class TrajectoryChunk:
    """A contiguous chunk of trajectory points for one worker process."""
    start: int
    points_xyz: np.ndarray
    orientations_abc: np.ndarray
    overlap: int = 0


_WORKER_SIM = None
_WORKER_T_BASE = None
_WORKER_T_TOOL_INV = None
_WORKER_SEED_TEMPLATES = ()


def _classify_solution(sim, solution):
    """Return unified status code from RobotSimulator evaluation."""
    evaluation = sim.evaluate_solution(solution)
    return int(evaluation.status)


def _resolve_target_transform(point_xyz, orientation_abc, t_base, t_tool_inv):
    t_point = kuka_base_to_matrix(
        point_xyz[0], point_xyz[1], point_xyz[2],
        orientation_abc[0], orientation_abc[1], orientation_abc[2]
    )
    t_flange_target = (t_base @ t_point) @ t_tool_inv
    target_pos = t_flange_target[0:3, 3] / 1000.0
    target_ori = t_flange_target[0:3, 0:3]
    return target_pos, target_ori


def _pick_initial_solution(sim, target_pos, target_ori, seed_templates):
    """Choose the best chunk-start seed among candidate templates."""
    if not seed_templates:
        fallback = np.zeros(len(sim.ik_chain.links), dtype=np.float64)
        seed_templates = (fallback,)

    best_seed = None
    best_solution = None
    best_status = 3  # UNREACHABLE

    for template in seed_templates:
        dyn_seed = sim.apply_base_azimuth_to_seed(template, target_pos)
        candidate_solution = sim.calculate_ik(
            target_pos,
            target_orientation=target_ori,
            initial_position=dyn_seed
        )
        candidate_status = _classify_solution(sim, candidate_solution)

        if best_seed is None or candidate_status < best_status:
            best_seed = dyn_seed
            best_solution = candidate_solution
            best_status = candidate_status
            if best_status == 0:  # OK
                break

    return best_seed, best_solution, best_status


def _init_worker(config: TrajectoryTestConfig):
    """Load robot/model state once per worker process."""
    from robot_ik import RobotSimulator

    global _WORKER_SIM, _WORKER_T_BASE, _WORKER_T_TOOL_INV, _WORKER_SEED_TEMPLATES

    sim = RobotSimulator()
    sim.load_robot(config.urdf_path)

    b_x, b_y, b_z, b_a, b_b, b_c = config.base_params
    t_x, t_y, t_z, t_a, t_b, t_c = config.tool_params
    _WORKER_SIM = sim
    _WORKER_T_BASE = kuka_base_to_matrix(b_x, b_y, b_z, b_a, b_b, b_c)
    t_tool = kuka_base_to_matrix(t_x, t_y, t_z, t_a, t_b, t_c)
    _WORKER_T_TOOL_INV = np.linalg.inv(t_tool)
    _WORKER_SEED_TEMPLATES = config.seed_templates


def _run_chunk_worker(chunk: TrajectoryChunk):
    """Worker entrypoint for a single chunk."""
    if _WORKER_SIM is None or _WORKER_T_BASE is None or _WORKER_T_TOOL_INV is None:
        raise RuntimeError("Trajectory test worker was not initialized.")

    n = len(chunk.points_xyz)
    statuses = np.zeros(n, dtype=np.int8)
    if n == 0:
        return chunk.start, statuses

    first_target_pos, first_target_ori = _resolve_target_transform(
        chunk.points_xyz[0], chunk.orientations_abc[0], _WORKER_T_BASE, _WORKER_T_TOOL_INV
    )
    best_seed, best_solution, best_status = _pick_initial_solution(
        _WORKER_SIM, first_target_pos, first_target_ori, _WORKER_SEED_TEMPLATES
    )
    statuses[0] = best_status

    prev_solution = best_solution if best_solution is not None else best_seed

    for i in range(1, n):
        target_pos, target_ori = _resolve_target_transform(
            chunk.points_xyz[i], chunk.orientations_abc[i], _WORKER_T_BASE, _WORKER_T_TOOL_INV
        )
        solution = _WORKER_SIM.calculate_ik(
            target_pos,
            target_orientation=target_ori,
            initial_position=prev_solution
        )
        statuses[i] = _classify_solution(_WORKER_SIM, solution)
        if solution is not None:
            prev_solution = solution

    if chunk.overlap > 0:
        statuses = statuses[chunk.overlap:]

    return chunk.start, statuses


def _split_chunks(points_xyz, orientations_abc, max_workers, chunk_size):
    total = len(points_xyz)
    if chunk_size is None:
        chunk_size = max(100, total // max_workers)

    chunks = []
    overlap = 5
    for start in range(0, total, chunk_size):
        end = min(start + chunk_size, total)
        chunk_start = max(0, start - overlap)
        chunk_overlap = start - chunk_start
        chunks.append(
            TrajectoryChunk(
                start,
                points_xyz[chunk_start:end],
                orientations_abc[chunk_start:end],
                overlap=chunk_overlap
            )
        )
    return chunks


def run_trajectory_test_parallel(points_xyz, orientations_abc, config: TrajectoryTestConfig, progress_callback=None):
    """Run full trajectory test in parallel and return status array."""
    if len(points_xyz) != len(orientations_abc):
        raise ValueError("points_xyz and orientations_abc must have equal length.")

    total = len(points_xyz)
    if total == 0:
        return np.zeros(0, dtype=np.int8)

    max_workers = config.max_workers
    if max_workers is None:
        max_workers = max(1, min(multiprocessing.cpu_count(), 8))

    chunks = _split_chunks(points_xyz, orientations_abc, max_workers, config.chunk_size)
    statuses = np.zeros(total, dtype=np.int8)
    completed = 0

    if progress_callback is not None:
        progress_callback(0, total)

    if max_workers == 1:
        _init_worker(config)
        for chunk in chunks:
            start, chunk_result = _run_chunk_worker(chunk)
            length = len(chunk_result)
            statuses[start:start + length] = chunk_result
            completed += length
            if progress_callback is not None:
                progress_callback(completed, total)
        return statuses

    with ProcessPoolExecutor(
        max_workers=max_workers,
        initializer=_init_worker,
        initargs=(config,),
    ) as pool:
        futures = {pool.submit(_run_chunk_worker, chunk): chunk for chunk in chunks}
        for future in as_completed(futures):
            start, chunk_result = future.result()
            length = len(chunk_result)
            statuses[start:start + length] = chunk_result
            completed += length
            if progress_callback is not None:
                progress_callback(completed, total)

    return statuses
