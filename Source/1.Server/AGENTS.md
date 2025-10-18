# Repository Guidelines

## Project Structure & Module Organization
The Flask streaming server lives in `server_highres_output.py` and owns YOLOv8 inference, credential checks, and frame broadcast. Edge device clients sit alongside it: `client_jpeg.py` handles JPEG push, while `client_cctv.py`, `client_low_buff.py`, `client_turtle.py`, and `client_user.py` cover tailored transports. Training helpers (`train_yolov8.py` and the scripts under `yolov5/`) assume Ultralytics layouts; keep datasets in `classification_dataset_prepared/`, `tomato_geti_dataset/`, or `yolo_dataset_prepared/`, and capture training artifacts under `runs/`. Reference `PROJECT_GUIDE.md` for deployment specifics and `requirements.txt` for Python dependencies.

## Build, Test, and Development Commands
Use `python3 server_highres_output.py` to launch the authenticated server and preview feeds at `/video_feed/<client_id>`. Trigger a JPEG uplink with `python3 client_jpeg.py --id <client> --password <secret>`; align credentials with `idlist.txt`. Train YOLOv8 locally via `python3 train_yolov8.py --data <config.yaml>`, and fall back to the vendor helpers for YOLOv5 (`python3 train_yolov5.py`). Run `python3 verify_labels.py` after pointing it to a sample batch to sanity-check annotations.

## Coding Style & Naming Conventions
Conform to PEP 8: four-space indentation, `snake_case` functions, `CamelCase` classes, and `UPPER_CASE` constants for network tokens or device IDs. Keep hardware or model configuration constants near the top of each module, mirror Ultralytics naming when editing files in `yolov5/`, and add concise docstrings when spawning new threads or sockets to clarify lifecycle expectations.

## Testing Guidelines
With no automated CI, run `python3 -m compileall server_highres_output.py client_jpeg.py` to catch syntax issues early. Manually pair the server with the relevant client script to confirm authentication, per-client feeds, FPS overlays, and detection callbacks before merging. After training, inspect the latest `runs/` artifacts and log any dataset conversions directly in your change notes.

## Commit & Pull Request Guidelines
Write imperative, scoped commits (e.g., “Add JPEG client retry logic”), and call out manual validation such as server–client smoke tests or training epochs. Pull requests should summarize behavioural changes, link tracking issues, and include screenshots or detection snippets whenever output frames or metrics change.

## Security & Configuration Tips
Store credentials as `id:password` pairs in `idlist.txt`, keep secrets out of history, and sync updates across every deployed client. Watch the server log for “인증 성공” after each credential change, rotate IDs and model weights before sharing hardware externally, and lean on environment variables for local path overrides.
