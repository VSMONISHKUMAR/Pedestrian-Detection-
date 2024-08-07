import cv2
import numpy as np
import serial
import time
import serial.tools.list_ports

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(port.device, port.description)
    return [port.device for port in ports]

def establish_serial_connection(port, baudrate, retries=10, delay=2):
    for attempt in range(retries):
        try:
            ser = serial.Serial(port, baudrate)
            time.sleep(2)  # Wait for the connection to establish
            return ser
        except serial.SerialException as e:
            print(f"Failed to connect to {port} on attempt {attempt + 1}/{retries}. Error: {e}")
            time.sleep(delay)
    raise serial.SerialException(f"Could not open port {port} after {retries} retries. Available ports: {list_serial_ports()}")

print("Available serial ports:")
available_ports = list_serial_ports()

# Ensure to use the correct port
port = 'COM8'  # Change this to the correct port identified from the list above
try:
    ser = establish_serial_connection(port, 9600)
except serial.SerialException as e:
    print(e)
    exit(1)

net = cv2.dnn.readNet("yolov2.weights", "yolov2.cfg")
classes = []

with open("coco.names", 'r') as f:
    classes = [line.strip() for line in f]

layer_names = net.getUnconnectedOutLayersNames()

def detect_objects(img):
    height, width, _ = img.shape

    blob = cv2.dnn.blobFromImage(img, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)

    outputs = net.forward(layer_names)

    boxes = []
    confidences = []
    class_ids = []

    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    person_detected = False  # Flag to check if a person is detected

    for i in indices:
        box = boxes[i]
        x, y, w, h = box
        label = classes[class_ids[i]]
        confidence = confidences[i]

        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(img, f'{label} {confidence:.2f}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Check if the detected object is a person
        if class_ids[i] == 0:  # 0 is the class ID for person in COCO dataset
            print("Person detected")
            person_detected = True

    # Send signal to Arduino based on detection
    if person_detected:
        ser.write(b'S')  # Send stop signal to Arduino
    else:
        ser.write(b'G')  # Send go signal to Arduino

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    detect_objects(frame)

    cv2.imshow('YOLO Object Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
