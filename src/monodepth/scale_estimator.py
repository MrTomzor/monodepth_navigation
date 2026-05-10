import numpy as np
from scipy.interpolate import griddata


import logging
logger = logging.getLogger(__name__)

SCALE_MIN = 0.01
SCALE_MAX = 1000.0



def get_scale_map(pointcloud_2d, depth_map, last_scale_map):
    points = []
    scales = []
    for (u, v, vins_depth) in pointcloud_2d:
        u = int(u)
        v = int(v)
        if (0 <= u < depth_map.shape[1]) and (0 <= v < depth_map.shape[0]):
            midas_depth = depth_map[v, u]
            if midas_depth > 0 and np.isfinite(midas_depth):
                scale = vins_depth / midas_depth
                if SCALE_MIN < scale < SCALE_MAX:
                    points.append([u, v])
                    scales.append(scale)

    points = np.array(points)
    scales = np.array(scales)

    H, W = depth_map.shape
    grid_x, grid_y = np.meshgrid(np.arange(W), np.arange(H))
    grid_coords = np.stack((grid_x, grid_y), axis=-1)

    if len(points) == 0:
        logger.warning("No points for interpolation, scale_map=last_scale_value")
        if last_scale_map is not None:
            return last_scale_map
        else:
            return np.ones((H, W), dtype=np.float32)
    elif len(points) < 3:
        logger.info("Using nearest-neighbor interpolation")
        return griddata(points, scales, grid_coords, method='nearest')
    else:
        scale_map_linear = griddata(points, scales, grid_coords, method='linear')
        scale_map_nearest = griddata(points, scales, grid_coords, method='nearest')
        return np.where(np.isnan(scale_map_linear), scale_map_nearest, scale_map_linear)



def get_scale_value(pointcloud_2d, depth_map, last_scale_value):
    pairs = []
    for (u, v, d) in pointcloud_2d:
        u = int(u)
        v = int(v)
        if (0 <= u < depth_map.shape[1]) and (0 <= v < depth_map.shape[0]):
            midas_depth = depth_map[v, u]
            if midas_depth > 0 and np.isfinite(midas_depth):
                pairs.append((midas_depth, d))

    if len(pairs) == 0:
        return last_scale_value

    abstract_depths = np.array([p[0] for p in pairs])
    real_depths = np.array([p[1] for p in pairs])

    new_scale = np.median(real_depths / abstract_depths)
    if np.isfinite(new_scale) and SCALE_MIN < new_scale < SCALE_MAX:
        return new_scale
    else:
        return last_scale_value