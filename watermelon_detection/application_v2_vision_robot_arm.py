import math
import threading
import time
import cv2
import numpy as np
import pyzed.sl as sl
from scipy.spatial.transform import Rotation as Rot
from ultralytics import YOLO
from dobot_api import DobotApi, DobotApiMove, DobotApiDashboard, MyType
from time import sleep

INIT_SCANNING_POS = -90
SCANNING_TRAVEL = 15
SCAN_DELAY = 2
CONF_VALUE = 0.5
INCH_TO_MM = 25.4

class ZEDYOLO3D:
    def __init__(self, yolo_model='/home/erin/Desktop/watermelon_detection/weights/small_v2.pt', robot_ip="192.168.5.1"):
        # Initialize ZED camera
        self.zed = sl.Camera()

        # Configure ZED camera parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 15
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.MILLIMETER

        # Open the camera
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("Failed to open ZED camera")
            exit(1)

        # Initialize runtime parameters
        self.runtime_parameters = sl.RuntimeParameters()

        # Initialize YOLO model
        self.model = YOLO(yolo_model)

        # Camera information
        self.camera_info = self.zed.get_camera_information()
        self.resolution = self.camera_info.camera_configuration.resolution

        # Create matrices for image and depth
        self.image_left = sl.Mat(self.resolution.width, self.resolution.height)
        self.depth_map = sl.Mat(self.resolution.width, self.resolution.height)
        self.point_cloud = sl.Mat(self.resolution.width, self.resolution.height)
        self.model.to('cuda')
        print("GPU: ", self.model.device)

        # Initialize arm
        self.robot_ip = robot_ip
        self.dashboard = None
        self.move = None
        self.feed = None
        self.connected = False
        self.close = None
        self.previous_com = ''

        # Real-time data
        self.current_position = None
        self.joint_angles = None
        self.feed_thread = None
        
        # Thread control
        self.auto_scan_thread = None
        self.stop_auto_scan = threading.Event()
        self.video_thread = None
        self.stop_video = threading.Event()
        self.auto_scan_active = False

        self.j1_harvest = 0

        self.connect_robot()

    def get_surface_normal_from_zed(self, x, y):
        """Get surface normal directly from ZED normal map"""
        try:
            # Retrieve normal map from ZED
            normal_map = sl.Mat()
            self.zed.retrieve_measure(normal_map, sl.MEASURE.NORMALS)

            # Get normal at the detected pixel coordinates
            err, normal = normal_map.get_value(x, y)

            if err == sl.ERROR_CODE.SUCCESS and np.isfinite(normal[:3]).all():
                # Normal is [nx, ny, nz, 0] - use first 3 components
                normal_vector = np.array([normal[0], normal[1], normal[2]])

                # Normalize just to be safe
                normal_length = np.linalg.norm(normal_vector)
                if normal_length > 0:
                    normal_vector = normal_vector / normal_length

                return normal_vector
            else:
                return None

        except Exception as e:
            print(f"ZED normal map error: {e}")
            return None

    def normal_to_euler_angles(self, normal_vector):
        """Convert surface normal vector to Euler angles (Rx, Ry, Rz) for robot"""
        try:
            # Desired Z-axis is the surface normal (suction cup perpendicular to surface)
            desired_z = normal_vector / np.linalg.norm(normal_vector)

            # Choose a reference direction for X-axis (could be robot's current X direction)
            # For simplicity, use global X-axis as reference, but project it to be perpendicular to normal
            reference_x = np.array([1, 0, 0])

            # If normal is parallel to reference, use different reference
            if np.abs(np.dot(desired_z, reference_x)) > 0.9:
                reference_x = np.array([0, 1, 0])

            # Calculate Y-axis (perpendicular to Z and X)
            desired_y = np.cross(desired_z, reference_x)
            desired_y = desired_y / np.linalg.norm(desired_y)

            # Recalculate X-axis to ensure orthogonality
            desired_x = np.cross(desired_y, desired_z)
            desired_x = desired_x / np.linalg.norm(desired_x)

            # Create rotation matrix
            rotation_matrix = np.column_stack([desired_x, desired_y, desired_z])

            # Convert to Euler angles (ZYX convention - common for robots)
            # Note: Dobot might use different convention - check your robot documentation
            rotation = Rot.from_matrix(rotation_matrix)
            euler_angles = rotation.as_euler('zyx', degrees=True)  # Z, Y, X order
            print("rotation matrix")
            # for row : rotation_matrix:

            print(rotation_matrix.shape)

            # Convert to Rx, Ry, Rz (Dobot convention - typically in degrees)
            # Dobot usually uses Rx, Ry, Rz in degrees
            rx = euler_angles[2]  # X rotation
            ry = euler_angles[1]  # Y rotation
            rz = euler_angles[0]  # Z rotation

            return rx, ry, rz

        except Exception as e:
            print(f"Euler angle conversion error: {e}")
            return 180, 0, 0  # Fallback to horizontal orientation

    def enhanced_pickup_watermelon(self, watermelon_object):
        """Enhanced pickup with surface normal alignment"""
        if not self.connected:
            print("Robot not connected")
            return False

        try:
            # Get watermelon center coordinates
            bbox = watermelon_object['bbox']
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)

            # Get surface normal at detection point
            surface_normal = self.get_surface_normal_from_zed(center_x, center_y)

            if surface_normal is not None:
                # Convert normal to robot Euler angles
                rx, ry, rz = self.normal_to_euler_angles(surface_normal)
                print(f"Surface normal alignment - Rx:{rx:.1f}, Ry:{ry:.1f}, Rz:{rz:.1f}")
            else:
                # Fallback to horizontal approach
                rx, ry, rz = 180, 0, 0
                print("Using horizontal approach (fallback)")

            # Get watermelon position
            watermelon_pos = watermelon_object['position_3d']

            # Transform to robot coordinates (using your measured transformation)
            # You'll need to implement this based on your camera-to-EE transformation
            robot_pos = self.transform_camera_to_robot(watermelon_pos)

            if robot_pos is None:
                print("Failed to transform coordinates")
                return False

            # Pickup positions
            approach_height = 90  # mm
            pickup_height = 10  # mm from surface

            # Approach position (above watermelon)
            approach_pos = robot_pos.copy()
            approach_pos[2] += approach_height

            # Pickup position (close to surface)
            pickup_pos = robot_pos.copy()
            pickup_pos[2] += pickup_height

            print("Starting enhanced pickup sequence...")

            # 1. Move to approach position with proper orientation
            print("1. Moving to approach position...")
            self.get_current_pose
            if not self.move_to_cartesian(approach_pos[0], approach_pos[1], approach_pos[2], rx, ry, rz):
                return False

            # 2. Move to pickup position (maintain orientation)
            print("2. Moving to pickup position...")
            self.get_current_pose
            if not self.move_to_cartesian(pickup_pos[0], pickup_pos[1], pickup_pos[2], rx, ry, rz, speed=10):
                return False

            # 3. Activate vacuu
            print("3. Activating vacuum...")
            self.activate_vacuum()
            sleep(1)

            # 4. Lift watermelon
            print("4. Lifting watermelon...")
            if not self.move_to_cartesian(approach_pos[0], approach_pos[1], approach_pos[2], rx, ry, rz):
                return False

            # 5. Move to bin (you can maintain orientation or use default)
            print("5. Moving to bin...")
            #if not self.move_to_bin():
            #    return False

            # 6. Release watermelon
            print("6. Releasing watermelon...")
            self.deactivate_vacuum()


            self.scanning_mode()

            print("✓ Enhanced pickup completed successfully!")
            return True

        except Exception as e:
            print(f"Enhanced pickup failed: {e}")
            self.deactivate_vacuum()
            return False

    def transform_camera_to_robot(self, camera_point):
        """Transform point from camera coordinates to robot coordinates"""
        # TODO: Implement using your measured transformation
        # This is where you use your camera-to-EE transformation matrix
        # For now, returning the point as-is (assuming coordinates are already in robot frame)
        """Transform point from camera coordinates to robot coordinates"""
        if self.current_position is None:
            print("No robot position available")
            return None

        try:
            # End-Effector to camera transformation matrix
            T_ee_to_camera = np.array([
                [0, 1, 0, -2.4 * INCH_TO_MM],
                [-1, 0, 0, +1.0 * INCH_TO_MM],
                [0, 0, 1, +1.7 * INCH_TO_MM],
                [0, 0, 0, 1]
            ])

            #End-Effector to Suction cup transformation matrix
            T_ee_to_suction = np.array([
                [1, 0, 0, 2.6 * INCH_TO_MM],
                [0, 1, 0, 0],
                [0, 0, 1, 4.9375 * INCH_TO_MM],
                [0, 0, 0, 1]
            ])

            # Convert camera point to homogeneous coordinates
            camera_point_h = np.array([camera_point[0], camera_point[1], camera_point[2], 1])

            # Transform watermelon point (from camera) → end-effector
            ee_point_h = camera_point_h @ np.linalg.inv(T_ee_to_camera)

            # Apply suction cup offset: ee → suction (subtract suction offset)
            suction_point_h = ee_point_h @ T_ee_to_suction
            print(suction_point_h) 

            # Get current end-effector pose and convert to transformation matrix
            current_pose = self.current_position
            T_base_to_ee = self.pose_to_transform_matrix(current_pose)

            # Transform robot base coordinates to suction point
            base_point_h = T_base_to_ee @ suction_point_h
            # Get current end-effector pose and convert to transformation matrix
            current_pose = self.current_position  # [x, y, z, rx, ry, rz] in robot base coordinates
            x, y, z, rx, ry, rz = current_pose

            # Create end-effector to base transformation matrix
            # This is the robot's current pose in base coordinates
            T_ee_to_base = self.pose_to_transform_matrix(current_pose)

            # Transform suction point to robot base coordinates
            print("After rotation")
            base_point_h = np.dot(T_ee_to_base, suction_point_h)
            print(base_point_h.shape)
            print(base_point_h)
            return base_point_h[:3]  # Return x, y, z

        except Exception as e:
            print(f"Transformation error: {e}")
            return None

    def pose_to_transform_matrix(self, pose):
        """Convert robot pose [x, y, z, rx, ry, rz] to 4x4 transformation matrix"""
        x, y, z, rx, ry, rz = pose

        # Convert to radians
        rx_rad = np.radians(rx)
        ry_rad = np.radians(ry)
        rz_rad = np.radians(rz)

        # Create rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx_rad), -np.sin(rx_rad)],
            [0, np.sin(rx_rad), np.cos(rx_rad)]
        ])

        Ry = np.array([
            [np.cos(ry_rad), 0, np.sin(ry_rad)],
            [0, 1, 0],
            [-np.sin(ry_rad), 0, np.cos(ry_rad)]
        ])

        Rz = np.array([
            [np.cos(rz_rad), -np.sin(rz_rad), 0],
            [np.sin(rz_rad), np.cos(rz_rad), 0],
            [0, 0, 1]
        ])

        # Combined rotation (ZYX convention typical for robots)
        R = Rz @ Ry @ Rx

        # 4x4 transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]

        print(R.shape)
        print(R)
        print(T.shape)
        print(T)

        return T

    def activate_vacuum(self):
        """Activate vacuum system"""
        print("VACUUM ACTIVATED")
        # TODO: Implement GPIO or Arduino control

    def deactivate_vacuum(self):
        """Deactivate vacuum system"""
        print("VACUUM DEACTIVATED")
        # TODO: Implement GPIO or Arduino control

    def process_frame(self):
        """Process a single frame"""
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve images and depth
            self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)

            # Convert ZED image to OpenCV format - FIXED COLOR CONVERSION
            image_ocv = self.image_left.get_data()
            
            # ZED returns BGRA format, convert to BGR for OpenCV
            if image_ocv.shape[2] == 4:
                image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_BGRA2BGR)

            # Run YOLO inference
            results = self.model(image_ocv, verbose=False)

            detected_objects = []

            for result in results:
                if result.masks is not None:
                    for mask, box in zip(result.masks.xy, result.boxes):
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = box.conf[0].cpu().numpy()
                        cls = int(box.cls[0].cpu().numpy())
                        class_name = self.model.names[cls]

                        # Calculate center point of bounding box
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)

                        # Get 3D position at center point
                        point3d = self.get_3d_position(center_x, center_y)

                        if point3d is not None:
                            detected_objects.append({
                                'class': class_name,
                                'confidence': conf,
                                'bbox': [x1, y1, x2, y2],
                                'position_3d': point3d,
                                'mask': mask
                            })

            return image_ocv, detected_objects
        return None, []

    def visualize_results(self, image, objects):
        """Visualize detection results with 3D information"""
        display_image = image.copy()

        for obj in objects:
            x1, y1, x2, y2 = map(int, obj['bbox'])
            class_name = obj['class']
            confidence = obj['confidence']
            pos_3d = obj['position_3d']

            # Draw bounding box
            cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw mask
            if obj['mask'] is not None:
                mask_points = obj['mask'].astype(np.int32)
                cv2.fillPoly(display_image, [mask_points], (150, 0, 0, 150))

            # Display information
            label = f"{class_name}: {confidence:.2f}"
            info = f"X:{pos_3d[0]:.2f}mm Y:{pos_3d[1]:.2f}mm Z:{pos_3d[2]:.2f}mm"

            cv2.putText(display_image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(display_image, info, (x1, y1 - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # Draw center point
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            cv2.circle(display_image, (center_x, center_y), 5, (0, 0, 255), -1)

        return display_image

    def connect_robot(self):
        """Connect to Dobot CR10"""
        try:
            print("Connecting to Dobot CR10...")
            self.dashboard = DobotApiDashboard(self.robot_ip, 29999)
            self.move = DobotApiMove(self.robot_ip, 30003)
            self.feed = DobotApi(self.robot_ip, 30004)

            # Enable robot
            self.dashboard.EnableRobot(0.45, 0, 0, 0)

            self.connected = True
            print("Dobot CR10 connected successfully!")

            # Start real-time data feed
            self.start_feed_thread()

        except Exception as e:
            print(f"Failed to connect to Dobot: {e}")
            self.connected = False

    def get_feed_data(self):
        """Thread function to get real-time robot data"""
        while self.connected:
            try:
                data = bytes()
                hasRead = 0
                while hasRead < 1440:
                    temp = self.feed.socket_dobot.recv(1440 - hasRead)
                    if len(temp) > 0:
                        hasRead += len(temp)
                        data += temp

                a = np.frombuffer(data, dtype=MyType)
                if hex((a['test_value'][0])) == '0x123456789abcdef':
                    self.current_position = a["tool_vector_actual"][0]
                    self.joint_angles = a["q_actual"][0]
                sleep(0.001)
            except Exception as e:
                print(f"Feed data error: {e}")
                break

    def start_feed_thread(self):
        """Start the real-time data feed thread"""
        self.feed_thread = threading.Thread(target=self.get_feed_data)
        self.feed_thread.setDaemon(True)
        self.feed_thread.start()
        print("Real-time data feed started")

    def get_current_pose(self):
        """Get and print current robot position"""
        if self.current_position is not None:
            print(f"Current Position: X={self.current_position[0]:.1f}, "
                  f"Y={self.current_position[1]:.1f}, Z={self.current_position[2]:.1f}, "
                  f"Rx={self.current_position[3]:.1f}, Ry={self.current_position[4]:.1f}, "
                  f"Rz={self.current_position[5]:.1f}")
            return self.current_position
        else:
            print("No position data available yet")
            return None

    def get_joint(self):
        """Get and print current joint position"""
        if self.joint_angles is not None:
            print(f"Current Joints: J1={self.joint_angles[0]:.1f}, "
                  f"J2={self.joint_angles[1]:.1f}, J3={self.joint_angles[2]:.1f}, "
                  f"J4={self.joint_angles[3]:.1f}, J5={self.joint_angles[4]:.1f}, "
                  f"J6={self.joint_angles[5]:.1f}")
            return self.joint_angles
        else:
            print("No position data available yet")
            return None

    def move_to_home(self):
        """Move to home position using joint coordinates"""
        if not self.connected:
            print("Robot not connected")
            return False

        try:
            print("Moving to HOME position...")
            # Home position in joint angles (degrees)
            self.dashboard.SpeedFactor(30)  # Slow speed for safety
            self.move.ServoJ(0, 0, 0, 0, 0, 180)
            sleep(5)  # Wait for movement to complete
            print("Reached home position")
            self.move.Sync()
            self.get_current_pose(
                
            )
            self.previous_com = 'h'
            return True
        except Exception as e:
            print(f"Home movement failed: {e}")
            return False


    def move_to_bin(self):
        """Move to bin position"""
        # TODO: Implement your bin coordinates
        bin_x, bin_y, bin_z = 200, 100, 100  # Example coordinates
        return self.move_to_cartesian(bin_x, bin_y, bin_z, 180, 0, 0)

    def move_to_cartesian(self, x, y, z, rx=0, ry=0, rz=0, speed=10):
        """Move to Cartesian position (ServoP)"""
        if not self.connected:
            print("Robot not connected")
            return False

        try:
            print(f"Moving to Cartesian position: X={x}, Y={y}, Z={z}")
            self.dashboard.SpeedFactor(speed)
            self.move.ServoP(x, y, z, rx, ry, rz)
            sleep(5)  # Wait for movement
            print("Movement completed")
            self.move.Sync()
            self.get_current_pose()

            return True
        except Exception as e:
            print(f"Cartesian movement failed: {e}")
            return False

    def move_joints(self, j1, j2, j3, j4, j5, j6, speed=10):
        """Move to joint angles (ServoJ)"""
        if not self.connected:
            print("Robot not connected")
            return False

        try:
            print(f"Moving joints: J1={j1}, J2={j2}, J3={j3}, J4={j4}, J5={j5}, J6={j6}")
            self.dashboard.SpeedFactor(speed)
            self.move.ServoJ(j1, j2, j3, j4, j5, j6)
            sleep(3)
            print("Joint movement completed")
            self.move.Sync()
            self.get_joint()
            self.previous_com = 'j'
            return True
        except Exception as e:
            print(f"Joint movement failed: {e}")
            return False

    def scanning_mode(self):
        print("Moving to scanning mode")
        self.move_joints(-90, -45, -45, 0, 91, 180)
        sleep(1)
        if self.current_position is not None:
            x, y, z, rx, ry, rz = self.current_position
            rx = 180
            ry = 0
            rz = 0
            """changed rx ry rz make end effector parallel to ground"""
            self.move_to_cartesian(x, y, z, rx, ry, rz)
            print("final position done")

    def auto_scan_helper(self):
        """Auto scan thread function"""
        reverse = 1
        while not self.stop_auto_scan.is_set():
            if self.joint_angles is not None:
                j1, j2, j3, j4, j5, j6 = self.joint_angles
                if (round(j1) <= -150):
                    reverse = 1
                elif (round(j1) >= -30):
                    reverse = -1
                j1 = j1 + reverse * SCANNING_TRAVEL
                self.move_joints(j1, j2, j3, j4, j5, j6)
            sleep(0.1)

            image, objects = self.process_frame()
            if objects:
                detected_watermelon = objects[0]  # Get first detected watermelon
            for watermelon in objects:
                if watermelon['confidence'] >= CONF_VALUE:
                    self.stop_auto_scan.set()  # Stop scanning
                    self.j1_harvest = self.get_joint()[0]
                    success = detector.enhanced_pickup_watermelon(watermelon)
                    sleep(0.5)
                    j1,j2,j3,j4,j5,j6 = self.get_joint()
                    self.move_joints(self.j1_harvest,j2,j3, j4, j5, j6)
                    self.stop_auto_scan.clear()  # Resume scanning
                break

    def start_auto_scan(self):
        """Start auto scan in a separate thread"""
        if self.auto_scan_active:
            print("Auto scan is already running!")
            return
            
        print("Starting auto scan...")
        self.move_joints(-90, 0, 0, 0, 45, 180)
        self.scanning_mode()
        self.stop_auto_scan.clear()
        self.auto_scan_active = True
        
        self.auto_scan_thread = threading.Thread(target=self.auto_scan_helper)
        self.auto_scan_thread.daemon = True
        self.auto_scan_thread.start()
        print("Auto scan started. Press 'a' to stop.")

    def stop_auto_scan_func(self):
        """Stop auto scan"""
        if not self.auto_scan_active:
            print("Auto scan is not running!")
            return
            
        print("Stopping auto scan...")
        self.stop_auto_scan.set()
        if self.auto_scan_thread:
            self.auto_scan_thread.join(timeout=2.0)
        self.auto_scan_active = False
        print("Auto scan stopped.")

    def video_display_loop(self):
        """Video display thread function"""
        cv2.namedWindow("ZED + YOLO 3D Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ZED + YOLO 3D Detection", 1280, 720)
        
        while not self.stop_video.is_set():
            image, objects = self.process_frame()

            if image is not None:
                # Visualize results
                display_image = self.visualize_results(image, objects)
                
                # Add status information to display
                status = "Auto Scan: " + ("ACTIVE" if self.auto_scan_active else "INACTIVE")
                cv2.putText(display_image, status, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(display_image, "Press 'a' to toggle auto-scan", (10, 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(display_image, "Press 't' for travel mode", (10, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(display_image, "Press 'q' to quit", (10, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow("ZED + YOLO 3D Detection", display_image)

            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.stop_video.set()
                break
            elif key == ord('a'):
                if self.auto_scan_active:
                    self.stop_auto_scan_func()
                else:
                    self.start_auto_scan()
                sleep(0.2)  # Debounce
            elif key == ord('t') and not self.auto_scan_active:
                print("Moving to travel mode...")
                self.travel_mode()

    def move_base(self):
        print("Moving to rotation mode")
        con = ''
        j1, j2, j3, j4, j5, j6 = self.joint_angles
        while True:
            self.get_joint()
            try:
                j1 = math.fmod(float(input("Enter J1 rotation ")), 360)
                self.move_joints(j1, j2, j3, j4, j5, j6)
            except ValueError:
                print("Invalid input! Please enter numbers only.")
            con = input("Continue Enter 'c'")
            if con != 'c':
                break

    def travel_mode(self):
        print("Moving to travel mode")
        self.move_joints(-90, 0, 0, 0, 45, 180)
        self.move_joints(-90, 0, -75, 90, 90, 180)
        self.move_joints(-90, 0, -156, 90, 90, 180)
        self.move_joints(-90, 0, -156, 162, 90, 180)
        self.move_joints(-90, 0, -156, 162, 90, 180)

    def get_3d_position(self, x, y):
        """Convert 2D pixel coordinates to 3D world coordinates"""
        err, point3d = self.point_cloud.get_value(x, y)
        if np.isfinite(point3d[0]) and np.isfinite(point3d[1]) and np.isfinite(point3d[2]):
            return point3d
        return None

    def run(self):
        """Main loop"""
        if not self.connected:
            print("Failed to connect to robot. Exiting.")
            return

        try:
            # Move to home first
            self.move_to_home()
            sleep(5)
            
            # Start video display thread
            self.video_thread = threading.Thread(target=self.video_display_loop)
            self.video_thread.daemon = True
            self.video_thread.start()
            #self.pose_to_transform_matrix(self.current_position)
            #self.get_current_pose
            print("System started!")
            print("Controls:")
            print("  'a' - Toggle auto-scan")
            print("  't' - Travel mode (when auto-scan is off)")
            print("  'q' - Quit program")
            
            # Wait for video thread to finish (when 'q' is pressed)
            self.video_thread.join()
            
        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up connections"""
        print("Cleaning up and disconnecting...")
        
        # Stop all threads
        self.stop_auto_scan.set()
        self.stop_video.set()
        self.connected = False
        
        # Wait for threads to finish
        if self.auto_scan_thread and self.auto_scan_thread.is_alive():
            self.auto_scan_thread.join(timeout=2.0)
        
        sleep(0.5)

        try:
            # Disable the robot first (safety)
            if self.dashboard:
                self.dashboard.DisableRobot()
            if self.feed:
                self.feed.socket_dobot.close()
            if self.move:
                self.move.socket_dobot.close()
            if self.dashboard:
                self.dashboard.socket_dobot.close()
            print("Robot disabled")
        except Exception as e:
            print(f"Error disabling robot: {e}")
        
        # Close ZED camera
        if self.zed:
            self.zed.close()
        
        # Close OpenCV windows
        cv2.destroyAllWindows()
        
        # Clear the objects
        self.dashboard = None
        self.move = None
        self.feed = None
        self.feed_thread = None

        print("All connections properly closed")

# Usage
if __name__ == "__main__":
    detector = ZEDYOLO3D('/home/erin/Desktop/watermelon_detection/weights/small_v2.pt', robot_ip="192.168.5.1")
    detector.run()

