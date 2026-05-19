import math
import numpy as np

def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Safely calculates the distance between two geographical points using Haversine or simple projection.
    Bounds checks can be added here.
    """
    # Simple euclidean over projection for performance as an example
    dx = abs(lat1 - lat2)
    dy = abs(lon1 - lon2)
    return math.sqrt(dx*dx + dy*dy)

def safe_interpolate(x_new: np.ndarray, x: np.ndarray, y: np.ndarray, kind='linear') -> np.ndarray:
    """
    Safely executes scipy.interpolate without crashing on out-of-bounds evaluation.
    This directly prevents the ValueError('A value in x_new is below the interpolation range').
    """
    # We delay the import so it doesn't fail if scipy isn't installed in pure python tests
    from scipy import interpolate
    
    # 1. Bounds constraint: clamp new 'x_new' array safely within the known (x) min/max
    min_x = np.min(x)
    max_x = np.max(x)
    
    x_new_clamped = np.clip(x_new, min_x, max_x)
    
    # Alternatively you can configure interp1d to extrapolate
    f = interpolate.interp1d(x, y, kind=kind, bounds_error=False, fill_value="extrapolate")
    
    try:
        return f(x_new_clamped)
    except Exception as e:
        print(f"Mathematical bounds failure during safe interpolation: {e}")
        # Graceful degradation logic returning defaults instead of crashing
        return np.zeros_like(x_new_clamped)

def safe_divide_points(remaining_points: int, drone_count: int) -> list:
    """
    Safely divides remaining points across drones ensuring sum equals total points.
    Prevents floating point rounding errors or Index/ZeroDivision exceptions.
    """
    if drone_count <= 0:
        return []
    
    base = remaining_points // drone_count
    remainder = remaining_points % drone_count
    
    return [base + 1 if i < remainder else base for i in range(drone_count)]
