# Project local configuration for Lidar/UDP/TCP
# Update these values to match your hardware/network settings.

# UDP frame buffer limit (bytes)
FRAME_BUF_SIZE = 4 * 1024 * 1024  # 4 MB

# UDP packet header offset used by the Lidar packet format
UDP_HEADER_OFFSET = 20

# Network settings for the LiDAR device (set to real IP/ports for production)
LIDAR_IP = "127.0.0.1"
TCP_PORT = 7686
UDP_PORT = 7687

# TCP command packet markers (must match the device protocol)
START_MARK = 0x12345678
END_MARK = 0x87654321
