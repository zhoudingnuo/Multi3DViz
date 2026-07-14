import open3d as o3d
import numpy as np
import os

# Thread limit is set by ccenter_app.py BEFORE open3d import.
# Kept here as a fallback if registration.py is imported standalone.
os.environ.setdefault("OMP_NUM_THREADS", str(max(2, (os.cpu_count() or 4) // 2)))

REGISTRATION_VOXEL_SIZE = 0.5
MIN_ICP = 0.2
MAX_RMSE = REGISTRATION_VOXEL_SIZE * 0.25 + MIN_ICP * 0.2
MIN_NUM_INLIERS = 200
MIN_FITNESS = 0.10
MIN_SCORE = 20
NUM_TRIALS = 5


def numpy2o3d(points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def compute_fgr(source_down, target_down, voxel_size):
    distance_threshold = voxel_size * 5.0
    fpfh_radius = voxel_size * 12.0
    search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_radius, max_nn=150)
    source_down.estimate_normals(search_param)
    target_down.estimate_normals(search_param)
    fpfh_source = o3d.pipelines.registration.compute_fpfh_feature(
        source_down, o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_radius, max_nn=150))
    fpfh_target = o3d.pipelines.registration.compute_fpfh_feature(
        target_down, o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_radius, max_nn=150))
    option = o3d.pipelines.registration.FastGlobalRegistrationOption(
        maximum_correspondence_distance=distance_threshold,
        iteration_number=100,
        division_factor=1.4)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, fpfh_source, fpfh_target, option)
    return result


def refine_icp(source, target, init_transform, voxel_size):
    result_icp = o3d.pipelines.registration.registration_icp(
        source, target, 1.0, init_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    result_icp = o3d.pipelines.registration.registration_icp(
        source, target, 0.5, result_icp.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    result_icp = o3d.pipelines.registration.registration_icp(
        source, target, MIN_ICP, result_icp.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result_icp


def icp_align(source_points, target_points, voxel_size=REGISTRATION_VOXEL_SIZE, T_init=np.eye(4),
              on_progress=None):
    """Align source to target using FGR + multi-stage ICP.
    Returns: (aligned_points, (fitness, rmse), is_valid, transformation)

    on_progress: optional callback called after each FGR+ICP trial with a dict:
        {'round': 1, 'trial': 1, 'fitness': 0.45, 'rmse': 0.12, 'score': 15.2,
         'n_inliers': 4500, 'best_score': 15.2, 'best_fitness': 0.45,
         'best_rmse': 0.12, 'is_valid': False, 'elapsed_s': 3.2, 'phase': 'try'}
      It is also called once at the start with phase='init' (after downsampling,
      carrying 'src_pts'/'tgt_pts' downsampled counts) so the UI can show the
      problem size before the first trial completes.
    """
    if len(source_points) < 100 or len(target_points) < 100:
        if on_progress:
            on_progress({'phase': 'abort', 'reason': 'too few points'})
        return source_points, (0.0, float('inf')), False, np.eye(4)

    pcd_source = numpy2o3d(source_points)
    pcd_target = numpy2o3d(target_points)

    if not np.allclose(T_init, np.eye(4)):
        pcd_source.transform(T_init)

    pcd_source_down = pcd_source.voxel_down_sample(voxel_size)
    pcd_target_down = pcd_target.voxel_down_sample(voxel_size)

    if len(pcd_source_down.points) < 10 or len(pcd_target_down.points) < 10:
        pcd_source_down = pcd_source
        pcd_target_down = pcd_target

    # Report problem size before the (slow) first trial.
    if on_progress:
        on_progress({'phase': 'init',
                     'src_pts': len(pcd_source_down.points),
                     'tgt_pts': len(pcd_target_down.points),
                     'max_trials': 3 * NUM_TRIALS})

    pcd_source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    pcd_target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    pcd_source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    pcd_target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

    import time as _t
    best_score = -1
    best_fitness = 0
    best_rmse = float('inf')
    best_transformation = np.eye(4)
    best_is_valid = False
    num_source = len(source_points)
    t_start = _t.time()

    for round_num in range(3):
        if best_is_valid:
            break
        for trial in range(NUM_TRIALS):
            fgr_result = compute_fgr(pcd_source_down, pcd_target_down, voxel_size)
            T_coarse = fgr_result.transformation @ T_init
            icp_result = refine_icp(pcd_source, pcd_target, T_coarse, voxel_size)

            fitness = icp_result.fitness
            rmse = icp_result.inlier_rmse
            num_inliers = int(fitness * num_source)
            score_fitness = fitness * 30.0
            score_rmse = (1.3 - min(rmse / MAX_RMSE, 1.0) - MIN_FITNESS) * 70.0
            score = score_fitness + score_rmse

            if score > best_score:
                best_score = score
                best_fitness = fitness
                best_rmse = rmse
                best_transformation = icp_result.transformation

            is_valid = (rmse <= MAX_RMSE and num_inliers >= MIN_NUM_INLIERS
                        and fitness >= MIN_FITNESS and score >= MIN_SCORE)
            if is_valid:
                best_is_valid = True

            elapsed = _t.time() - t_start
            status = "PASS" if is_valid else "FAIL"
            print(f"  R{round_num+1}/T{trial+1}: fitness={fitness:.4f} rmse={rmse:.4f} "
                  f"score={score:.1f} [{status}] ({elapsed:.1f}s)")
            if on_progress:
                on_progress({'phase': 'try',
                             'round': round_num + 1, 'trial': trial + 1,
                             'fitness': fitness, 'rmse': rmse, 'score': score,
                             'n_inliers': num_inliers,
                             'best_score': best_score, 'best_fitness': best_fitness,
                             'best_rmse': best_rmse, 'is_valid': is_valid,
                             'elapsed_s': elapsed})

        if best_is_valid:
            print(f"  Valid result found at round {round_num+1}")
            break

    if best_score <= 0:
        if on_progress:
            on_progress({'phase': 'done', 'ok': False, 'elapsed_s': _t.time() - t_start})
        return source_points, (best_fitness, best_rmse), False, np.eye(4)

    if not best_is_valid:
        print(f"  All attempts failed validation, using best (score={best_score:.1f})")

    if on_progress:
        on_progress({'phase': 'done', 'ok': best_is_valid,
                     'best_score': best_score, 'best_fitness': best_fitness,
                     'best_rmse': best_rmse, 'elapsed_s': _t.time() - t_start})

    source_hom = np.hstack([source_points, np.ones((source_points.shape[0], 1))])
    aligned = (best_transformation @ source_hom.T).T[:, :3]
    return aligned, (best_fitness, best_rmse), best_is_valid, best_transformation
