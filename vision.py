import math
import time
from modules import detector_mobilenet as detector
from modules import control

# Stereo vision constants
BASELINE = 0.06  # 6 cm
FOCAL_LENGTH_PX = 800  # Calibrated focal length in pixels
CAMERA_FOV = 60  # in degrees

def get_single_axis_delta(value1, value2):
    return value2 - value1

def point_in_rectangle(point, left, right, top, bottom):
    return left < point[0] < right and top < point[1] < bottom

def estimate_distance_from_box(box_height, image_height, known_person_height_m=1.7, focal_length_px=600):
    if box_height <= 0:
        return 10.0
    return (known_person_height_m * focal_length_px) / box_height

def estimate_distance_stereo(center1, center2, image_width):
    """
    Estimates depth using synthetic stereo from two object center positions.
    """
    x1 = center1[0]
    x2 = center2[0]
    disparity = abs(x1 - x2)
    if disparity == 0:
        return None

    depth = (FOCAL_LENGTH_PX * BASELINE) / disparity

    # Also compute horizontal angle offset from image center
    angle_deg = ((x1 - image_width / 2) / image_width) * CAMERA_FOV
    angle_rad = math.radians(angle_deg)

    dx = depth * math.tan(angle_rad)
    return depth, dx

def autonomous_stereo_estimate():
    """
    Detect object, move drone sideways, re-detect, estimate depth.
    Returns depth and lateral offset (dx) if successful, else None.
    """
    print(" Detecting object for stereo vision...")
    detections1, _, image1 = detector.get_detections()
    if not detections1:
        print(" No object detected in first frame.")
        return None

    center1 = detections1[0].Center
    image_width = image1.shape[1]

    # Move drone right by 6 cm
    print(" Moving drone right by 6cm...")
    control.control_drone()  # maintain flight control
    control.setXdelta(-0.06)  # right in BODY_NED
    control.control_drone()
    time.sleep(1.5)  # wait for drone to move

    # Capture second frame
    detections2, _, image2 = detector.get_detections()
    if not detections2:
        print(" No object detected in second frame.")
        return None

    center2 = detections2[0].Center

    # Estimate depth
    result = estimate_distance_stereo(center1, center2, image_width)
    if result:
        depth, dx = result
        print(f" Estimated depth: {depth:.2f}m | Horizontal offset: {dx:.2f}m")
        return depth, dx
    else:
        print(" Disparity too small, depth estimation failed.")
        return None
