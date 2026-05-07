# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Project Is

A real-time industrial safety monitoring desktop application for a steel manufacturing yard. It fuses 5 RTSP camera streams with a 3D LiDAR sensor, runs YOLO object detection (person / truck / hook) via ONNX Runtime on GPU, and displays a PySide6 GUI with configurable ROI safety zones, a live top-down map, and a Normal / Warning / Danger state machine with alarm popups and event-triggered video recording.

## Running the Application

```bash
python main_gy.py
```

No build step. The working directory is set to the script location at startup (`os.chdir`), so all relative paths (DB files, weight files, output folders) are resolved from the project root.

## Key Configuration

`config.py` — edit before running on real hardware:
- `LIDAR_IP`, `TCP_PORT` (7686), `UDP_PORT` (7687) — LiDAR network settings
- `START_MARK` / `END_MARK` — TCP packet framing bytes (must match device protocol)
- `FRAME_BUF_SIZE`, `UDP_HEADER_OFFSET` — UDP packet assembly parameters

RTSP camera URLs and ROI coordinates are stored in `setting_gy.db` (not in source files). They are edited through the Settings dialog at runtime.

## Model Conversion

The runtime uses an ONNX model, not the raw PyTorch weights:

```bash
python pt_to_onnx.py   # converts weights/gy_best.pt → weights/gy_best.onnx
```

Three classes: `0=person`, `1=truck`, `2=hook` (defined in `gun_young.yaml` and hardcoded in `YOLO_ONNX`).

## Architecture Overview

### Entry Point and Main Window

`main_gy.py` → creates `QApplication` + `widget` (from `widget_gy.py`).

`widget_gy.py` hosts the main window logic:
- Loads ROI coordinates and camera settings from `setting_gy.db` at startup
- Owns the safety state machine (Normal → Warning → Danger) driven by a `QTimer`
- Draws the top-down map showing detected object positions across all cameras
- Handles cross-camera duplicate filtering for overlapping FOVs (cam2↔cam3, cam3↔cam4, cam1, cam5)
- `D` key forces Danger state for testing (`test_force_danger` flag)
- ROI intrusion detection uses line equations: each ROI zone is 4 points; the boundary lines are computed by `_get_line_equation` / `_calculate_roi_line_equations` and stored as `(A, B, C)` tuples (standard line form `Ax + By + C = 0`)

### Camera Pipeline (`camera_gy.py`)

```
FrameGrabber (QThread, one per camera)
    → grabs RTSP frames continuously
    → puts into a thread-safe queue

VideoWorker (QThread, one per camera)
    → pulls frames from queue
    → runs YOLO_ONNX inference
    → emits detection results + annotated frame via Qt signals
```

`YOLO_ONNX` wraps ONNX Runtime. The model is a **singleton** loaded once in `CameraController` and shared across all `VideoWorker` instances to avoid GPU memory duplication. CUDA is preferred; falls back to CPU automatically.

`CustomMapLabel` is a `QLabel` subclass that paints the top-down map.

### LiDAR Pipeline

```
LidarWorker (QObject moved to QThread)
    ├── tcp_client.py  — TCP socket: sends start/stop commands (0x01 / 0x02)
    └── udp_client.py  — UDP socket: receives depth frame packets, reassembles them
                         (double-buffer + threading.Event for sync)
```

`LidarWorker.run()` has an auto-reconnect outer loop (5-second retry on connection failure). It emits `frame_ready` (QImage for display) and `frame_shared` (numpy BGR array for `EnlargeDialog`). Depth data goes through IQR outlier removal and spike suppression before 8-bit normalization.

UDP packets are assembled via `assemble_packet` in `udp_client.py` — each LiDAR frame is split into multiple UDP packets identified by `image_sn` (frame ID); all are collected into `packet_data` dict before assembling.

### Dialogs

- **`EnlargeDialog`** (`enlarge_image.py`) — popup for a single enlarged camera or LiDAR feed; supports mouse-wheel zoom + drag-pan. LiDAR mode lets the user click to measure depth (mm). Camera mode uses `VideoWorker` or shares the existing `FrameGrabber` worker.
- **`AlarmDialogManager`** (`alarm_gy.py`) — alarm history viewer backed by `alarm_gy.db`; supports date-range filtering, CSV export, and clicking an alarm row to view its snapshot image.
- **`RecordingManager`** (`recording_manager.py`) — `QObject`-based; handles `cv2.VideoWriter` lifecycle and emits signals for start/stop/error. Codec: `mp4v`, output: `video_recording/`.

### UI Files

`.ui` files (Qt Designer) are **pre-compiled** to `*ui.py` Python files. After editing a `.ui` file, regenerate the Python file:

```bash
pyside6-uic mainwindow_gy.ui -o mainwindowgyui.py
pyside6-uic alarm_dlg.ui    -o alarm_dlgui.py
pyside6-uic setting_dlg.ui  -o setting_dlgui.py
pyside6-uic enlarge_image.ui -o enlarge_imageui.py
```

### Databases

| File | Table | Purpose |
|------|-------|---------|
| `alarm_gy.db` | `Alarm` | Event log: `al_date`, `al_cam`, `al_grade` (`warning`/`danger`), `al_file` (image path) |
| `setting_gy.db` | various | RTSP URLs, ROI point lists (stored as Python list strings, parsed with `ast.literal_eval`), LiDAR calibration (`X_dist`, `Y_dist`, `x_ref`) |

Reset alarm history (keeps schema):
```bash
python clean_db.py
```

### Runtime Output Directories (auto-created)

| Directory | Contents |
|-----------|----------|
| `video_recording/` | MP4 recordings |
| `image_capture/` | Manual PNG snapshots |
| `warning_images/` | Auto-captured warning event frames |
| `danger_images/` | Auto-captured danger event frames |

## Threading Model

All workers are `QObject` instances moved to `QThread` with `moveToThread()` — **not** `QThread` subclasses. Cross-thread communication is exclusively via Qt signals/slots. `Qt.QueuedConnection` is used wherever a slot updates UI from a worker thread. Never call Qt UI methods directly from a worker thread.
