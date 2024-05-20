"""
STOP SIGN DETECTION BOT

This script captures video frames and uses OpenCV to detect stop signs. For each frame processed,
it measures and logs the time taken to process the frame. Detected stop signs trigger a message sent
over UART to a TIVA board, indicating the detection status, which is part of the STOP SIGN DETECTION BOT system.
After processing all frames or upon termination, it logs the average and worst-case computation times.

Authors: Kiran Jojare, Ayswariya Kannan
Subject: ECEN 5623 Real-Time Embedded Systems
Professor: Tim Scherr
University: University of Colorado Boulder
References:
    - https://github.com/maurehur/Stop-Sign-detection_OpenCV
    - https://devpost.com/software/stop-sign-detection

This code is part of an academic project under the Real-Time Embedded Systems course at the University
of Colorado Boulder, focusing on the practical application of real-time concepts in embedded systems
and the integration with embedded hardware like the TIVA board for real-world applications.
"""

import cv2
import numpy as np
import time
import syslog
import serial

# Configure the serial port
ser = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
)

# Load stop sign detection classifier
stop_sign_cascade = cv2.CascadeClassifier("/home/rtes/Desktop/cascade_stop_sign.xml")

# Configure the video capture
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Initialize previous detection state
prev_detected = False

# List to store computation times
frame_times = []

try:
    while True:
        start_time = time.time()  # Start time of the frame processing

        # Read frame from camera
        ret, frame = cap.read()
        if not ret:
            break

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect stop signs
        stop_signs = stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Determine detection status
        detected = len(stop_signs) > 0

        # Log detection
        if detected:
            syslog.syslog(syslog.LOG_INFO, "Stop sign detected")

        # Send data over UART when detection status changes
        if detected != prev_detected:
            data_packet = bytes([0xAA] * 8) if detected else bytes([0x00] * 8)
            ser.write(data_packet)
            syslog.syslog(syslog.LOG_INFO, f"Sent {'0xAA' if detected else '0x00'} packet over UART")
            prev_detected = detected

        # Display detected stop signs
        for (x, y, w, h) in stop_signs:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.imshow("Frame", frame)

        end_time = time.time()  # End time of the frame processing
        frame_time = end_time - start_time
        frame_times.append(frame_time)  # Append frame processing time
        syslog.syslog(syslog.LOG_INFO, f"Frame Computation Time: {frame_time:.4f} seconds")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

    # Calculate average and worst case execution times
    if frame_times:
        average_time = sum(frame_times) / len(frame_times)
        worst_case_time = max(frame_times)
        syslog.syslog(syslog.LOG_INFO, f"Average Frame Computation Time: {average_time:.4f} seconds")
        syslog.syslog(syslog.LOG_INFO, f"Worst Case Frame Computation Time: {worst_case_time:.4f} seconds")

