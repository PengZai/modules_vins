import pyzed.sl as sl

# Create a ZED camera object
zed = sl.Camera()

# Set initialization parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD1080  # Or use HD1080, HD2K, etc.
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE   # Optional, not critical for calibration
init_params.coordinate_units = sl.UNIT.METER         # Ensures baseline is returned in meters

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"Failed to open camera: {err}")
    exit(1)

# Get calibration parameters
calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters

# Intrinsics of the left camera
left_cam = calibration_params.left_cam
print("=== Left Camera Intrinsics ===")
print(f"Resolution: {left_cam.image_size.width} x {left_cam.image_size.height}")
print(f"Focal Length (fx, fy): {left_cam.fx}, {left_cam.fy}")
print(f"Principal Point (cx, cy): {left_cam.cx}, {left_cam.cy}")
print(f"Distortion Coefficients: {left_cam.disto}")  # [k1, k2, p1, p2, k3]
print(f"Horizontal FOV: {left_cam.h_fov}°")
print(f"Vertical FOV: {left_cam.v_fov}°")
print(f"Diagonal FOV: {left_cam.d_fov}°")

# Extrinsics (baseline)
stereo_transform = calibration_params.stereo_transform
translation = stereo_transform.get_translation().get()  # [tx, ty, tz]
baseline = translation[0]
print("\n=== Extrinsics (Left-Right) ===")
print(f"Translation (x, y, z): {translation} meters")
print(f"Baseline (tx): {baseline} meters")

# Optional: get intrinsic matrix
K = [
    [left_cam.fx, 0, left_cam.cx],
    [0, left_cam.fy, left_cam.cy],
    [0, 0, 1]
]
print("\n=== Camera Matrix (K) ===")
for row in K:
    print(row)

# Close the camera
zed.close()