FROM ros:jazzy

# 1. Aggiornamento sistema, strumenti base e compilatori per Python
RUN apt update -y && apt install -y wget git nano python3-pip python3-dev build-essential libgl1 libglib2.0-0

# 2. Strumenti grafici di debug per ROS 2
RUN apt update -y && apt install -y ros-jazzy-rqt-graph ros-jazzy-rqt-tf-tree

# 3. AMBIENTE E ROBOT: Gazebo Harmonic e URDF/Xacro
RUN apt update -y && apt install -y ros-jazzy-ros-gz ros-jazzy-xacro

# 4. NAVIGAZIONE: Nav2 Stack
RUN apt update -y && apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# 5. LOCALIZZAZIONE E SENSORI: EKF e AprilTags
RUN apt update -y && apt install -y ros-jazzy-robot-localization ros-jazzy-apriltag-ros

# 6. VISIONE ARTIFICIALE: OpenCV e YOLOv8
RUN apt update -y && apt install -y ros-jazzy-vision-opencv
RUN pip3 install ultralytics --break-system-packages