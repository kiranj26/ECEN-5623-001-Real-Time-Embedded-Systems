# Real Time Stop Sign Detection for Vehicles using FreeRTOS and TIVA Launchpad Board

## Project Overview
This project develops a real-time embedded system for autonomous vehicles, utilizing the TIVA C123GX microcontroller and NVIDIA Jetson Nano to detect and respond to STOP signs. Designed as a navigational aid, this bot enhances safety in intelligent transportation systems through advanced image processing and machine learning.

## Key Features
- **High-Performance Processing:** Utilizes NVIDIA Jetson Nano's GPU and CPU for real-time video analysis with OpenCV.
- **Efficient Object Detection:** Employs Haar Cascade classifiers to recognize traffic signs.
- **Robust Communication:** Features UART with Cyclic Redundancy Checks ensuring data integrity between the Jetson Nano and TIVA board.
- **Motor Control:** PWM signals from TIVA C123GX control motor drivers based on detection inputs.
- **Real-Time Operating System:** FreeRTOS on TIVA C123GX facilitates multitasking and prioritizes tasks to meet real-time operational deadlines.
- **Scheduling and Analysis:** Uses Rate-Monotonic Scheduling (RMS) and Cheddar Analysis to ensure responsiveness and system integrity.

## Technologies Used
- TIVA C123GX Microcontroller
- NVIDIA Jetson Nano
- Logitech C310 HD Webcam
- OpenCV Library
- FreeRTOS
- PWM Motor Control
- UART Communication

## Get Started
To get started with this project, please clone the repository and refer to the documentation provided in the `docs` folder for setup and operational guidance.

## Contributing
Contributors are welcome! Please read the `CONTRIBUTING.md` for the process of submitting pull requests to us.

## License
Distributed under the MIT License. See `LICENSE` for more information.

## Contact
For more information, queries, or to get involved, please contact us via GitHub or directly through our project email.
