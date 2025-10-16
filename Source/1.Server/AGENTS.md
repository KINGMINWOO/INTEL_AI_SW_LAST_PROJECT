# Repository Guidelines

## Project Structure & Module Organization
The real-time streaming stack sits at the root: `server_highres_output.py` hosts YOLOv8 inference with Flask streaming, while `client.py` and `client_jpeg.py` push frames from edge devices. Training helpers (`train_model.sh`, `train_yolov5.py`, `train_yolov8.py`) wrap the vendor `yolov5/` directory. Datasets and label tools live in `classification_dataset_prepared/`, `tomato_geti_dataset/`, and `yolo_dataset_prepared/`, with rendered artifacts under `runs/`; Jetson/DeepStream experiments stay in `jetson/`.

## Build, Test, and Development Commands
Run `python3 server_highres_output.py` to start the authenticated server; monitor individual uplinks at `/video_feed/<client_id>`. Use `python3 client_jpeg.py` for the JPEG uplink or `python3 client.py` for the pickle stream; both expect credentials from `idlist.txt`. Train via `bash train_model.sh` or `python3 train_yolov5.py` (YOLOv5) and `python3 train_yolov8.py` (YOLOv8). Validate labels with `python3 verify_labels.py`, adjusting the sample paths before use.

## Coding Style & Naming Conventions
Follow Python 3 PEP 8: four-space indentation, snake_case for functions, and UPPER_CASE for constants such as handshake tokens or IPs. Mirror Ultralytics naming when editing `yolov5/` scripts, and keep hardware configuration near the top of each module. Brief docstrings on new threads or sockets help other agents understand lifecycle boundaries.

## Testing Guidelines
No CI yet; run `python3 -m compileall <file>` for quick syntax checks. Pair `server_highres_output.py` with `client_jpeg.py` to confirm authentication, client-specific streams (`/video_feed/<client_id>`), FPS overlay, and detection callbacks before merging. After training, review metrics and sample frames in `runs/`, and document any dataset or label conversions in your PR.

## Commit & Pull Request Guidelines
Keep commits small, imperative (e.g., “Add credential handshake”), and scoped. Note manual validation (server/client pairing, training epoch) in commit bodies or PRs. Each PR should link tracking issues, summarize behaviour changes, and include screenshots or detection snippets when outputs change.

## Security & Configuration Tips
Manage credentials in `idlist.txt` using `id:password`; avoid committing production secrets. Update matching values in each client before deployment and watch the server log for “인증 성공” to confirm access. Rotate IDs and weights when sharing with external devices, and prefer environment overrides for local paths.
