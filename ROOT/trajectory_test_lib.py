import multiprocessing
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass

import numpy as np

from .pose_math import kuka_base_to_matrix


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
class TrajectoryTestProgress:
    """Structured progress event for long-running trajectory tests."""

    stage: str
    completed: int
    total: int
    detail: str = ""


@dataclass(frozen=True)
class TrajectoryChunk:
    """A contiguous chunk of trajectory points for one worker process."""

    start: int
    points_xyz: np.ndarray
    orientations_abc: np.ndarray
    initial_position: np.ndarray | None = None


_WORKER_SIM = None
_WORKER_T_BASE = None
_WORKER_T_TOOL_INV = None
_WORKER_SEED_TEMPLATES = ()


def _classify_solution(sim, solution):
    evaluation = sim.evaluate_solution(solution)
    return int(evaluation.status)


def _resolve_target_transform(point_xyz, orientation_abc, t_base, t_tool_inv):
    t_point = kuka_base_to_matrix(
        point_xyz[0],
        point_xyz[1],
        point_xyz[2],
        orientation_abc[0],
        orientation_abc[1],
        orientation_abc[2],
    )
    t_flange_target = (t_base @ t_point) @ t_tool_inv
    target_pos = t_flange_target[0:3, 3] / 1000.0
    target_ori = t_flange_target[0:3, 0:3]
    return target_pos, target_ori


def _pick_initial_solution(sim, target_pos, target_ori, seed_templates):
    if not seed_templates:
        seed_templates = (np.zeros(len(sim.ik_chain.links), dtype=np.float64),)

    best_seed = None
    best_solution = None
    best_status = 3

    for template in seed_templates:
        dyn_seed = sim.apply_base_azimuth_to_seed(template, target_pos)
        candidate_solution = sim.calculate_ik(
            target_pos,
            target_orientation=target_ori,
            initial_position=dyn_seed,
        )
        candidate_status = _classify_solution(sim, candidate_solution)

        if best_seed is None or candidate_status < best_status:
            best_seed = dyn_seed
            best_solution = candidate_solution
            best_status = candidate_status
            if best_status == 0:
                break

    return best_seed, best_solution, best_status


def _init_worker(config: TrajectoryTestConfig):
    from .robot_ik import RobotSimulator

    global _WORKER_SIM, _WORKER_T_BASE, _WORKER_T_TOOL_INV, _WORKER_SEED_TEMPLATES

    sim = RobotSimulator()
    sim.load_robot(config.urdf_path, load_meshes=False)

    b_x, b_y, b_z, b_a, b_b, b_c = config.base_params
    t_x, t_y, t_z, t_a, t_b, t_c = config.tool_params
    _WORKER_SIM = sim
    _WORKER_T_BASE = kuka_base_to_matrix(b_x, b_y, b_z, b_a, b_b, b_c)
    _WORKER_T_TOOL_INV = np.linalg.inv(kuka_base_to_matrix(t_x, t_y, t_z, t_a, t_b, t_c))
    _WORKER_SEED_TEMPLATES = config.seed_templates


def _run_chunk_worker(chunk: TrajectoryChunk):
    if _WORKER_SIM is None or _WORKER_T_BASE is None or _WORKER_T_TOOL_INV is None:
        raise RuntimeError("Trajectory test worker was not initialized.")

    point_count = len(chunk.points_xyz)
    statuses = np.zeros(point_count, dtype=np.int8)
    if point_count == 0:
        return chunk.start, statuses

    first_target_pos, first_target_ori = _resolve_target_transform(
        chunk.points_xyz[0],
        chunk.orientations_abc[0],
        _WORKER_T_BASE,
        _WORKER_T_TOOL_INV,
    )

    if chunk.initial_position is None:
        best_seed, best_solution, best_status = _pick_initial_solution(
            _WORKER_SIM, first_target_pos, first_target_ori, _WORKER_SEED_TEMPLATES
        )
        prev_solution = best_solution if best_solution is not None else best_seed
        statuses[0] = best_status
    else:
        solution = _WORKER_SIM.calculate_ik(
            first_target_pos,
            target_orientation=first_target_ori,
            initial_position=chunk.initial_position,
        )
        statuses[0] = _classify_solution(_WORKER_SIM, solution)
        prev_solution = solution if solution is not None else chunk.initial_position

    for index in range(1, point_count):
        target_pos, target_ori = _resolve_target_transform(
            chunk.points_xyz[index],
            chunk.orientations_abc[index],
            _WORKER_T_BASE,
            _WORKER_T_TOOL_INV,
        )
        solution = _WORKER_SIM.calculate_ik(
            target_pos,
            target_orientation=target_ori,
            initial_position=prev_solution,
        )
        statuses[index] = _classify_solution(_WORKER_SIM, solution)
        if solution is not None:
            prev_solution = solution

    return chunk.start, statuses


def _emit_progress(progress_callback, stage, completed, total, detail=""):
    if progress_callback is not None:
        progress_callback(
            TrajectoryTestProgress(
                stage=stage,
                completed=int(completed),
                total=int(total),
                detail=detail,
            )
        )


def _split_chunks(points_xyz, orientations_abc, max_workers, chunk_size):
    total = len(points_xyz)
    if chunk_size is None:
        target_chunk_count = max(1, max_workers * 4)
        chunk_size = max(100, min(500, (total + target_chunk_count - 1) // target_chunk_count))

    chunks = []
    for start in range(0, total, chunk_size):
        end = min(start + chunk_size, total)
        chunks.append(TrajectoryChunk(start, points_xyz[start:end], orientations_abc[start:end]))
    return chunks


def _build_chunk_seed_handoff(points_xyz, orientations_abc, config: TrajectoryTestConfig, chunks, progress_callback=None):
    """Sequential seed pass that hands the previous chunk's terminal solution to the next chunk."""
    if len(chunks) <= 1:
        return chunks

    from .robot_ik import RobotSimulator

    simulator = RobotSimulator()
    simulator.load_robot(config.urdf_path, load_meshes=False)

    b_x, b_y, b_z, b_a, b_b, b_c = config.base_params
    t_x, t_y, t_z, t_a, t_b, t_c = config.tool_params
    t_base = kuka_base_to_matrix(b_x, b_y, b_z, b_a, b_b, b_c)
    t_tool_inv = np.linalg.inv(kuka_base_to_matrix(t_x, t_y, t_z, t_a, t_b, t_c))

    boundary_indices = {chunk.start for chunk in chunks if chunk.start > 0}
    handoff_seeds = {}
    prev_solution = None
    report_interval = max(1, len(points_xyz) // 200)

    for index in range(len(points_xyz)):
        if index in boundary_indices:
            handoff_seeds[index] = None if prev_solution is None else np.array(prev_solution, dtype=np.float64, copy=True)

        target_pos, target_ori = _resolve_target_transform(points_xyz[index], orientations_abc[index], t_base, t_tool_inv)
        if prev_solution is None:
            best_seed, best_solution, _ = _pick_initial_solution(simulator, target_pos, target_ori, config.seed_templates)
            prev_solution = best_solution if best_solution is not None else best_seed
            continue

        solution = simulator.calculate_ik(
            target_pos,
            target_orientation=target_ori,
            initial_position=prev_solution,
        )
        if solution is not None:
            prev_solution = solution
        if progress_callback is not None and ((index + 1) % report_interval == 0 or index + 1 == len(points_xyz)):
            progress_callback(index + 1, len(points_xyz))

    return [
        TrajectoryChunk(
            start=chunk.start,
            points_xyz=chunk.points_xyz,
            orientations_abc=chunk.orientations_abc,
            initial_position=handoff_seeds.get(chunk.start),
        )
        for chunk in chunks
    ]


def run_trajectory_test_parallel(points_xyz, orientations_abc, config: TrajectoryTestConfig, progress_callback=None):
    """Run full trajectory test and return one status code per point."""
    if len(points_xyz) != len(orientations_abc):
        raise ValueError("points_xyz and orientations_abc must have equal length.")

    total = len(points_xyz)
    if total == 0:
        return np.zeros(0, dtype=np.int8)

    max_workers = config.max_workers
    if max_workers is None:
        max_workers = max(1, min(multiprocessing.cpu_count(), 8))

    chunks = _split_chunks(points_xyz, orientations_abc, max_workers, config.chunk_size)
    max_workers = max(1, min(max_workers, len(chunks)))
    statuses = np.zeros(total, dtype=np.int8)
    completed = 0
    needs_seed_handoff = max_workers > 1 and len(chunks) > 1
    overall_total = total * 2 if needs_seed_handoff else total

    _emit_progress(progress_callback, "prepare", 0, overall_total, "Starting trajectory test...")

    if max_workers == 1:
        _init_worker(config)
        for chunk in chunks:
            start, chunk_result = _run_chunk_worker(chunk)
            length = len(chunk_result)
            statuses[start:start + length] = chunk_result
            completed += length
            _emit_progress(
                progress_callback,
                "solve",
                completed,
                overall_total,
                f"Testing points: {completed} / {total}",
            )
        return statuses

    seeded_chunks = _build_chunk_seed_handoff(
        points_xyz,
        orientations_abc,
        config,
        chunks,
        progress_callback=lambda prepared, prepared_total: _emit_progress(
            progress_callback,
            "seed",
            prepared,
            overall_total,
            f"Preparing chunk seeds: {prepared} / {prepared_total}",
        ),
    )
    _emit_progress(
        progress_callback,
        "spawn",
        total,
        overall_total,
        f"Starting {max_workers} worker processes...",
    )

    with ProcessPoolExecutor(
        max_workers=max_workers,
        initializer=_init_worker,
        initargs=(config,),
    ) as pool:
        futures = {pool.submit(_run_chunk_worker, chunk): chunk for chunk in seeded_chunks}
        for future in as_completed(futures):
            start, chunk_result = future.result()
            length = len(chunk_result)
            statuses[start:start + length] = chunk_result
            completed += length
            _emit_progress(
                progress_callback,
                "solve",
                total + completed,
                overall_total,
                f"Testing points: {completed} / {total}",
            )

    return statuses
