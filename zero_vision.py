# zero_computer_vision_uart.py
# Run this on the Raspberry Pi Zero
# Install dependencies:
#   sudo apt update
#   sudo apt install python3-picamera2 python3-pip
#   pip3 install imagezmq numpy opencv-python>=4.0.0 degirum pyserial simplejpeg

import socket
import time
import serial
from picamera2 import Picamera2
import imagezmq
import cv2    
import numpy as np
import simplejpeg
import degirum as dg

# test if the script can run automatically
with open("/home/CPSPi/boot_log.txt", "a") as f:
    f.write("Script started\n")

# UART Configuration (must match Pico settings)
SERIAL_PORT = '/dev/serial0'  # or '/dev/ttyS0'
BAUD_RATE = 115200

# DeGirum Configuration
DEGIRUM_TOKEN = "token here"

def setup_degirum():
    """Initialize DeGirum model for object detection"""
    try:
        # Load local model for faster inference
        model = dg.load_model(
            model_name="yolov5m_relu6_coco--640x640_quant_tflite_multidevice_1",
            inference_host_address="@local",
            zoo_url='degirum/public',
            token=DEGIRUM_TOKEN,
        )
        print("DeGirum model loaded successfully")
        return model
    except Exception as e:
        print(f"Failed to load DeGirum model: {e}")
        return None

def setup_uart():
    """Initialize UART connection to Pico"""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"UART connection established on {SERIAL_PORT}")
        return ser
    except Exception as e:
        print(f"Failed to setup UART: {e}")
        return None

def send_detection_message(ser, detected_objects):
    """Send detection results to Pico via UART"""
    if ser is None:
        return
    
    try:
        # Send a simple message when objects are detected
        if detected_objects:
            message = "object_detected\n"
            time.sleep(0.1) 
            ser.write(message.encode('utf-8'))
            print(f"Sent to Pico: {message.strip()}")
            for obj in detected_objects:
                if obj.get('type') == 'qr_code':
                    message = f"object_detected:QR_CODE:{obj.get('data', 'unknown')}\n"
                elif 'label' in obj:  # DeGirum detection
                    message = f"object_detected:{obj['label'].upper()}:{obj.get('score', 0):.2f}\n"
                else:
                    message = "object_detected:UNKNOWN:0.0\n"
                
                ser.write(message.encode('utf-8'))
                print(f"Sent to Pico: {message.strip()}")
                time.sleep(0.1)  # 
    except Exception as e:
        print(f"UART send error: {e}")

def process_degirum_results(results, confidence_threshold=0.):
    """Process DeGirum detection results and return detected objects"""
    detected_objects = []
    
    try:
        if hasattr(results, 'results') and results.results:
            for detection in results.results:
                # Check if detection has required attributes
                if hasattr(detection, 'score') and detection.score > confidence_threshold:
                    obj_info = {
                        'label': getattr(detection, 'label', 'unknown'),
                        'score': detection.score,
                        'bbox': getattr(detection, 'bbox', None)
                    }
                    detected_objects.append(obj_info)
                    print(f"Detected: {obj_info['label']} (confidence: {obj_info['score']:.2f})")
    except Exception as e:
        print(f"Error processing DeGirum results: {e}")
    
    return detected_objects

def squareness(bbox):
    """Calculate how square a bounding box is (from your original code)"""
    if bbox is not None:
        points = bbox[0].astype(int)
        side1 = np.linalg.norm(points[0] - points[1])
        side2 = np.linalg.norm(points[1] - points[2])
        side3 = np.linalg.norm(points[2] - points[3])
        side4 = np.linalg.norm(points[3] - points[0])
        
        if min(side1, side2, side3, side4) > 0:
            max_side = max(side1, side2, side3, side4)
            min_side = min(side1, side2, side3, side4)
            side_ratio = min_side / max_side
        else:
            side_ratio = 0
    else:
        side_ratio = 0
    return side_ratio

def object_is_visible(qr_bbox, frame_width):
    """Check if object is visible (from your original code)"""
    if qr_bbox is None:
        return False
    
    points = qr_bbox[0].astype(int)
    
    for point in points:
        if point[0] >= frame_width or point[0] < 0:
            return False
    
    top_width = np.linalg.norm(points[0] - points[1])
    bottom_width = np.linalg.norm(points[3] - points[2])
    
    if top_width <= 0 or bottom_width <= 0:
        return False
    
    qr_estimated_width = (top_width + bottom_width) / 2
    rightmost_x = max(points[0][0], points[1][0], points[2][0], points[3][0])
    landmark_width_factor = 1.2
    estimated_landmark_right_x = rightmost_x + (qr_estimated_width * landmark_width_factor)
    
    return estimated_landmark_right_x < frame_width

def main():
    # Initialize components
    detector = cv2.QRCodeDetector()
    degirum_model = setup_degirum()
    uart_connection = setup_uart()
    
    # Setup image streaming (optional - for debugging)
    try:
        connect_to = "tcp://192.168.50.237:5555"  # Update with your laptop IP
        sender = imagezmq.ImageSender(connect_to=connect_to)
        rpi_name = socket.gethostname()
        streaming_enabled = True
    except:
        print("Image streaming disabled - will only do local processing")
        streaming_enabled = False #这行原本在except里面，如果要实时看到
        sender = None
    
    # Initialize camera
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(preview_config)
    picam2.start()
    time.sleep(2)  # Let camera warm up
    
    print("Computer vision system started. Press Ctrl+C to stop.")
    
    try:
        frame_count = 0
        while True:
            frame = picam2.capture_array()
            if frame.shape[2] == 4: 
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            detected_objects = []
            
            # QR Code Detection (simplified - any QR code triggers message)
            data, bbox, _ = detector.detectAndDecode(frame)
            if data:
                print(f"QR Code detected: {data}")
                # Add to detected objects regardless of shape/position
                detected_objects.append({'type': 'qr_code', 'data': data})
                print("✓ Sending detection message for QR code")
                
                # Optional: Still draw bounding box with quality indicators
                if bbox is not None:
                    points = bbox[0].astype(int)
                    frame_width = frame.shape[1]
                    object_in_view = object_is_visible(bbox, frame_width)
                    is_square = squareness(bbox) > 0.8
                    
                    # Color coding for visual feedback (but doesn't affect message sending)
                    if object_in_view and is_square:
                        box_color = (0, 255, 0)  # Green: Perfect
                    elif object_in_view and not is_square:
                        box_color = (0, 255, 255)  # Yellow: Distorted but visible
                    elif not object_in_view and is_square:
                        box_color = (0, 165, 255)  # Orange: Square but edge case
                    else:
                        box_color = (0, 0, 255)  # Red: Poor quality
                    
                    cv2.polylines(frame, [points], isClosed=True, color=box_color, thickness=2)
                    upper_left = tuple(points[0])
                    cv2.circle(frame, upper_left, radius=6, color=(0, 0, 255), thickness=-1)
            
            # DeGirum Object Detection (every few frames to save processing)
            if degirum_model is not None and frame_count % 10 == 0:  # Process every 10th frame
                try:
                    results = degirum_model(frame)
                    degirum_objects = process_degirum_results(results)
                    detected_objects.extend(degirum_objects)
                except Exception as e:
                    print(f"DeGirum inference error: {e}")
            
            # Send detection results to Pico via UART
            if detected_objects:
                send_detection_message(uart_connection, detected_objects)
            
            # Stream to laptop (optional)
            if streaming_enabled and sender is not None:
                try:
                    jpg_buffer = simplejpeg.encode_jpeg(frame, quality=95, colorspace='BGR')
                    sender.send_jpg(rpi_name, jpg_buffer)
                except Exception as e:
                    print(f"Streaming error: {e}")
            
            frame_count += 1
            time.sleep(0.1)  # Small delay to prevent overwhelming the system
            
    except KeyboardInterrupt:
        print("Stopping computer vision system...")
    finally:
        picam2.stop()
        if uart_connection:
            uart_connection.close()
        if sender:
            sender.close()

if __name__ == "__main__":
    main()
