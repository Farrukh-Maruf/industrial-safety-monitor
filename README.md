# Steel Yard Safety Monitor

Real-time industrial safety monitoring system for a steel manufacturing yard. Fuses 5 RTSP camera streams with a 3D LiDAR sensor, runs YOLO object detection via ONNX Runtime on GPU, and displays a PySide6 desktop GUI with configurable safety zones, a live top-down map, and automated alarms with video recording.

---

## Features

- **5-camera RTSP fusion** — simultaneous live feeds with per-camera ROI safety zones
- **3D LiDAR integration** — TCP command + UDP depth stream with auto-reconnect
- **YOLO object detection** — 3 classes: `person`, `truck`, `hook` (ONNX Runtime, CUDA preferred)
- **Safety state machine** — Normal → Warning → Danger with alarm popups
- **Live top-down map** — shows detected object positions across all cameras
- **Event recording** — auto-captures snapshots and MP4 video clips on Warning/Danger
- **Alarm history** — SQLite log with date-range filtering and CSV export
- **Configurable ROI zones** — edit via Settings dialog at runtime, stored in database

---

## Requirements

- Python 3.10+
- CUDA-capable GPU (recommended; falls back to CPU automatically)
- RTSP cameras accessible on the network
- LiDAR device on a reachable IP

---

## Installation

```bash
git clone <repo-url>
cd gonyon_steel_source

python -m venv .venv
.venv\Scripts\activate          # Windows
# source .venv/bin/activate     # Linux/macOS

pip install -r requirements.txt
```

Key packages: `PySide6`, `onnxruntime-gpu`, `opencv-python`, `torch`, `ultralytics`, `numpy`

---

## Configuration

Edit `config.py` before running on real hardware:

```python
LIDAR_IP   = "192.168.x.x"   # LiDAR device IP
TCP_PORT   = 7686
UDP_PORT   = 7687
START_MARK = 0x12345678       # TCP framing bytes — must match device protocol
END_MARK   = 0x87654321
```

RTSP camera URLs and ROI zone coordinates are stored in `setting_gy.db` and edited through the **Settings dialog** at runtime (no need to touch source files).

---

## Model Setup

The runtime requires an ONNX model. Convert from PyTorch weights:

```bash
# Place your trained weights at weights/gy_best.pt
python pt_to_onnx.py
# Output: weights/gy_best.onnx
```

Classes are fixed: `0=person`, `1=truck`, `2=hook`.

---

## Running

```bash
python main_gy.py
```

The app sets its working directory to the project root on startup, so all relative paths resolve correctly regardless of where you launch from.

**Debug shortcut:** press `D` to force Danger state for testing.

---

## Project Structure

```
gonyon_steel_source/
├── main_gy.py            # Entry point
├── widget_gy.py          # Main window, state machine, top-down map
├── camera_gy.py          # FrameGrabber + VideoWorker + YOLO_ONNX
├── lidar_worker.py       # LiDAR TCP/UDP pipeline
├── tcp_client.py         # TCP command socket
├── udp_client.py         # UDP depth frame assembler
├── enlarge_image.py      # Zoom/pan popup dialog (camera + LiDAR)
├── alarm_gy.py           # Alarm history dialog
├── recording_manager.py  # Video recording lifecycle
├── clean_db.py           # Reset alarm history
├── config.py             # Hardware / network settings
├── gun_young.yaml        # YOLO class definitions
├── weights/              # Model weights (not tracked in git)
│   ├── gy_best.pt
│   └── gy_best.onnx
├── alarm_gy.db           # Alarm event log (auto-created)
├── setting_gy.db         # Camera URLs + ROI settings (not tracked in git)
├── video_recording/      # Auto-created — MP4 recordings
├── image_capture/        # Auto-created — manual PNG snapshots
├── warning_images/       # Auto-created — warning event frames
└── danger_images/        # Auto-created — danger event frames
```

---

## UI Files

`.ui` files (Qt Designer) are pre-compiled to `*ui.py`. After editing a `.ui` file, regenerate:

```bash
pyside6-uic mainwindow_gy.ui  -o mainwindowgyui.py
pyside6-uic alarm_dlg.ui      -o alarm_dlgui.py
pyside6-uic setting_dlg.ui    -o setting_dlgui.py
pyside6-uic enlarge_image.ui  -o enlarge_imageui.py
```

---

## Databases

| File | Table | Purpose |
|------|-------|---------|
| `alarm_gy.db` | `Alarm` | Event log: date, camera, grade (`warning`/`danger`), snapshot path |
| `setting_gy.db` | various | RTSP URLs, ROI point lists, LiDAR calibration values |

Reset alarm history (keeps schema):

```bash
python clean_db.py
```

---

## Architecture Overview

```
main_gy.py
└── widget_gy.py  (main window + state machine)
    ├── camera_gy.py  (per-camera pipeline)
    │   ├── FrameGrabber  [QThread] — RTSP grab loop
    │   └── VideoWorker   [QThread] — YOLO inference + signal emit
    └── lidar_worker.py  [QObject → QThread]
        ├── tcp_client.py — start/stop commands
        └── udp_client.py — depth frame assembly
```

All workers use `moveToThread()` with Qt signals/slots for cross-thread communication. UI is never touched from worker threads.