import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO

class ZEDYOLO3D:
    def __init__(self, yolo_model='/home/erin/Desktop/watermelon_detection/weights/small_v2.pt'):
    #def __init__(self, yolo_model='yolo11n-seg.pt'):
        # Initialize ZED camera
        self.zed = sl.Camera()
        
        # Configure ZED camera parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 15
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        
        
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
        self.image_left = sl.Mat(self.resolution.width/2, self.resolution.height/2)
        self.depth_map = sl.Mat(self.resolution.width/2, self.resolution.height/2)
        self.point_cloud = sl.Mat(self.resolution.width/2, self.resolution.height/2)
        self.model.to('cuda')
        print(self.model.device)
        
    def get_3d_position(self, x, y, depth_value):
        """Convert 2D pixel coordinates to 3D world coordinates"""
        err, point3d = self.point_cloud.get_value(x, y)
        if np.isfinite(point3d[0]) and np.isfinite(point3d[1]) and np.isfinite(point3d[2]):
            return point3d
        return None
    
    def process_frame(self):
        """Process a single frame"""
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve images and depth
            self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            
            # Convert ZED image to OpenCV format
            image_ocv = self.image_left.get_data()
            image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGB2BGR)
            image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2RGB)
            
            
            # Run YOLO inference
            results = self.model(image_ocv)
            
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
                        point3d = self.get_3d_position(center_x, center_y, 0)
                        
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
                cv2.fillPoly(display_image, [mask_points], (150, 0, 0,150))
            
            # Display information
            label = f"{class_name}: {confidence:.2f}"
            info = f"X:{pos_3d[0]:.2f}m Y:{pos_3d[1]:.2f}m Z:{pos_3d[2]:.2f}m"
            
            cv2.putText(display_image, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(display_image, info, (x1, y1-30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Draw center point
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            cv2.circle(display_image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        return display_image
    
    def run(self):
        """Main loop"""
        try:
            while True:
                image, objects = self.process_frame()
                
                if image is not None:
                    # Visualize results
                    display_image = self.visualize_results(image, objects)
                    cv2.imshow("ZED + YOLO 3D Detection", cv2.resize(display_image,(1280, 720)))
                    #cv2.imshow("RGB Image", image)
                    
                    # Print 3D positions
                    for obj in objects:
                        print(f"{obj['class']}: {obj['position_3d']}")
                
                # Exit on 'q' press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.zed.close()
            cv2.destroyAllWindows()

# Usage
if __name__ == "__main__":
    detector = ZEDYOLO3D('/home/erin/Desktop/watermelon_detection/weights/small_v2.pt')
    #detector = ZEDYOLO3D('yolo11n-seg.pt')
    detector.run()
