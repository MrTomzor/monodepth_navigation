import numpy as np
from scipy.interpolate import griddata


def get_scale_map(pointcloud_2d, depth_map):
    points = []
    scales = []
    for (u, v, vins_depth) in pointcloud_2d:
        u = int(u)
        v = int(v)
        if (0 <= u < depth_map.shape[1]) and (0 <= v < depth_map.shape[0]):
            midas_depth = depth_map[v, u]
            if midas_depth > 0 and np.isfinite(midas_depth):
                scale = vins_depth / midas_depth
                if 1e-5 < scale < 1e8:
                    points.append([u, v])
                    scales.append(scale)

    points = np.array(points)
    scales = np.array(scales)

    H, W = depth_map.shape
    grid_x, grid_y = np.meshgrid(np.arange(W), np.arange(H))
    grid_coords = np.stack((grid_x, grid_y), axis=-1)

    if len(points) == 0:
        print("WARNING: No points available for interpolation")
        return np.ones(depth_map.shape, dtype=np.float32)
    elif len(points) < 3:
        print("INFO: Not enough points for Delaunay-based linear interpolation — using nearest only")
        scale_map = griddata(points, scales, grid_coords, method='nearest')
    else:
        print("INFO: Using Delaunay-based linear interpolation")
        scale_map_linear = griddata(points, scales, grid_coords, method='linear')
        scale_map_nearest = griddata(points, scales, grid_coords, method='nearest')
        scale_map = np.where(np.isnan(scale_map_linear), scale_map_nearest, scale_map_linear)

    return scale_map


def get_scale_value(pointcloud_2d, depth_map, points_count=3):
    pairs = []
    for (u, v, d) in pointcloud_2d:
        u = int(u)
        v = int(v)
        if (0 <= u < depth_map.shape[1]) and (0 <= v < depth_map.shape[0]):
            midas_depth = depth_map[v, u]
            if midas_depth > 0 and np.isfinite(midas_depth):
                pairs.append((midas_depth, d))

    if len(pairs) == 0:
        return 1

    pairs.sort(key=lambda x: x[1])
    closest_pairs = pairs[:points_count]

    abstract_depths = np.array([p[0] for p in closest_pairs])
    real_depths = np.array([p[1] for p in closest_pairs])

    new_scale = np.median(real_depths / abstract_depths)
    if np.isfinite(new_scale) and 0.01 < new_scale < 1000.0:
        return new_scale
    else:
        return 1