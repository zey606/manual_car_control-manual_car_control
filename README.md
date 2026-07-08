# Manuel Kontrollü LiDAR Haritalama Aracı

Bu proje DENEYAP kart ile manuel kontrol edilen ve T3 Gemstone üzerinde ROS 2 kullanarak haritalama yapan mobil araç projesidir.

## Proje dosyaları

- imu_guard: DENEYAP ve Gemstone IMU verilerinin işlenmesi
- imu_publisher: ICM-20948 IMU verisinin ROS 2 ortamında yayınlanması
- lidar_uyandir.py: RPLIDAR seri bağlantısının hazırlanması

## Kullanılan sistemler

- ROS 2 Humble
- RPLIDAR A1M8
- RF2O Laser Odometry
- SLAM Toolbox
- RViz 2
