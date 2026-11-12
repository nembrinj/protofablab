# Walking video experiment

A Python experiment for walking analysis and lower-body pose model comparison.
The offline workflow estimates step candidates and cadence and exports an annotated
video. The live workflow streams a USB webcam through your computer's browser to
Python in the devcontainer and compares eight pretrained pose model options.

Start with [the model guide](docs/MODELS.md) for intended uses, exact download
sources, checkpoint hashes, score definitions, and preliminary camera results.

| Workflow | Notebook | Models | Results |
| --- | --- | --- | --- |
| Saved-video walking analysis | [01_walk_experiment](notebooks/01_walk_experiment.ipynb) | MediaPipe task bundles; Lite by default | Candidate steps, cadence, plots, debug video/CSV |
| Live camera comparison | [02_live_webcam_pose](notebooks/02_live_webcam_pose.ipynb) | MediaPipe Lite/Full/Heavy, YOLOv8n, YOLO11n, MoveNet Thunder, RTMPose-m, RTMO-m | Skeleton overlay and per-frame text logs |

The five additional options are a practical comparison set, not a measured
popularity ranking. All include hips, knees and ankles; only the MediaPipe options
used here include heels and foot tips. None has yet been validated for legs-only
camera framing.

**Status:** useful experimental baseline. Step-count accuracy has not been validated
against manually labeled recordings. A visible pose is not proof that a counted
candidate is a real step.

## Quick start

1. Install Docker Desktop, VS Code, and the **Dev Containers** extension.
2. Open this folder and run **Dev Containers: Reopen in Container**.
3. The container installs the local package in editable mode automatically. In an
   existing container, install it once from the project root:

```sh
python -m pip install --no-build-isolation -e .
```

4. In the container terminal, run:

```sh
python -m walk_analysis check
python -m walk_analysis download-model
```

Place a video at `data/walk.mp4`, or download the default URL with:

```sh
python -m walk_analysis download-video
```

The downloader also supports VS Code's **Run Python File** button: open
`src/walk_analysis/download_video.py` and run it directly. The equivalent terminal
command is:

```sh
python src/walk_analysis/download_video.py
```

Both launch methods use `DEFAULT_URL` from that file when no URL is supplied and
save to `data/walk.mp4` by default. Previous experiment URLs are kept as comments
above `DEFAULT_URL`. Direct-file execution is supported specifically for the
downloader; use the package commands below for the other tools.

To select another URL or output path without editing source:

```sh
python -m walk_analysis download-video 'https://www.youtube.com/watch?v=VIDEO_ID' --output data/walk.mp4
```

The downloader uses yt-dlp, Deno for YouTube JavaScript challenges, and FFmpeg to
merge video and audio into MP4. A bundled FFmpeg is supplied by `imageio-ffmpeg`
when a system executable is unavailable. An existing output may be reused by
yt-dlp: move the old recording elsewhere before downloading a different URL to
`data/walk.mp4`.

Open [notebooks/01_walk_experiment.ipynb](notebooks/01_walk_experiment.ipynb),
select the `/usr/local/bin/python` kernel, and choose **Restart Kernel → Run All**.
The cells inspect the video, collect ankle tracks, plot signals, print metrics,
plot numbered candidates, evaluate tracking quality and timing, and generate an inline debug video. The final video
cell can also run independently; it reprocesses the video from disk.

Use one person, a fixed side-view camera, the full body in frame, good lighting,
and visible feet. The code analyzes one pose and does not maintain person identity
when multiple people appear.

## Download pose models

```sh
python -m walk_analysis download-model --model all
```

Downloads all eight pose options plus the RTMPose person detector to `models/`.
Use `--model mediapipe` for only the Lite, Full, and Heavy float16 v1 bundles.
Use `--model full` or `--model heavy` for one model. With no arguments the
command still downloads Lite. `--output PATH` selects a single output file;
`--output-dir DIR` selects the directory for any model selection.

The five alternative test options need additional CPU backends:

```sh
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements-pose.txt
python -m pip install --no-deps rtmlib==0.0.16
python -m walk_analysis download-model --model alternatives
```

RTMLib's dependencies are supplied separately to retain headless OpenCV. Restart
the notebook kernel after installation. Native confidence/visibility scores differ
between model families and should not be interpreted as equal accuracy measures.
COCO models supply hips, knees and ankles, but no heels/toes. The notebook describes
person selection and the application threshold used to reject weak predictions.

## Live USB webcam test

Open [notebooks/02_live_webcam_pose.ipynb](notebooks/02_live_webcam_pose.ipynb)
and run the cells one at a time. The notebook serves a browser camera preview on
port 8765: forward that port privately in VS Code and open it in your computer's
external browser. Enable the camera, select the Logitech USB device, and start the
pose test. Frames are sent to the selected local pose model in
the devcontainer; USB passthrough is not required. The page shows a skeleton,
pose coverage, and native ankle scores. Choose MediaPipe, YOLOv8n Pose, YOLO11n
Pose, MoveNet Thunder, RTMPose-m, or RTMO-m in the browser (download models before
running the notebook). Each detection log line records
the selected model. Stop the camera and run the cleanup cell
when finished. This tests human pose detection, not general object detection.

## Command-line analysis

Run these commands from the project root:

```sh
python -m walk_analysis analyze data/walk.mp4
python -m walk_analysis debug
```

Both analysis and debug commands accept a video path and `--model PATH` for a
MediaPipe `.task` bundle. These offline commands do not use the optional YOLO,
MoveNet, RTMPose, or RTMO adapters; those are currently live-notebook only.
The debug command also accepts `--output-dir PATH`. Defaults point to this source
checkout, so the installed commands can run from another working directory.
The notebook defaults to `data/walk.mp4`.

```sh
python -m walk_analysis debug 'old videos/first video/walk.mp4' --output-dir outputs/first-video
python -m walk_analysis analyze --help
```

After installation, `walk-analysis` is an equivalent command-line entry point.

| Output | Contents |
| --- | --- |
| `outputs/walk_debug.mp4` | Silent H.264 preview, pose lines, ankle visibility, running count, and yellow candidate flashes labeled L/R |
| `outputs/step_candidates.csv` | Candidate number, zero-based source frame, time in seconds, both ankle heights, triggering foot, and prominence |

The exporter replaces these debug outputs on each run. The video is scaled to
960 pixels wide. Frame timestamps are computed from frame index and reported FPS;
variable-frame-rate sources are not handled using their original presentation times.
Generated videos and downloaded models are excluded from Git and the Docker build
context; keep recordings you want to preserve in separate storage.

## Model and counting algorithm

**Offline default model:** MediaPipe Pose Landmarker Lite, downloaded by the
`download-model` command with no arguments. Full and Heavy `.task` bundles can also
be selected for offline analysis. The live notebook supports all eight options
described in [the model guide](docs/MODELS.md). MediaPipe
returns body landmarks per frame, including normalized ankle coordinates and
visibility scores. Image y increases downward, so the counter inverts ankle height
to turn foot lifts into peaks. L/R refer to the person's anatomical left and right.

**Algorithm:** `walk_analysis.cadence` processes each foot separately. It rejects unreliable
ankles, splits tracks at missing or unreliable samples and long time gaps, smooths
each segment, and finds local foot-lift peaks. It suppresses peaks too close
together using timestamps, then combines the two feet. It does not force strict
left/right alternation. The notebook, CLI metrics, plot, CSV, and video use this
same candidate detector when given the same tracks and visibility values.

Defaults in `detect_step_candidates`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `min_visibility` | 0.5 | Minimum ankle visibility score |
| `smoothing_s` | 0.10 s | Full width of the centered averaging window |
| `max_gap_s` | 0.20 s | Largest time gap allowed within a segment |
| `min_same_foot_s` | 0.40 s | Minimum separation between peaks for one foot within a segment |
| `min_step_s` | 0.18 s | Minimum separation between combined candidates |
| `min_prominence` | 0.003 | Minimum peak prominence in normalized image height; an adaptive threshold can increase this |

These are starting values, not calibrated settings for every person or camera.
A lift candidate precedes ground contact. Peaks at segment boundaries can be missed
because the detector needs samples on both sides. Jitter, occlusion, camera motion,
small foot lifts, and landmark swaps can cause missed or extra counts.

**Cadence** is candidate count divided by the time between the first and last
pose-detected frames, multiplied by 60. This window includes internal tracking gaps;
it is not necessarily the full clip duration or the time spent actively walking.

**Speed** is disabled by default. The optional `--meters-per-normalized-height`
flag (or notebook variable `METERS_PER_NORMALIZED_HEIGHT`) multiplies the net
horizontal displacement of the ankle midpoint by a scene scale and divides by
the observation duration. It is a rough net image-displacement estimate, not total
walking distance or validated gait speed. Turning around can produce a near-zero
estimate despite substantial walking. Perspective and camera motion also affect it.
Do not assume a person's height alone calibrates horizontal floor distances.

## How to validate a recording

1. Play the debug video slowly and count individual foot contacts manually. One
   left contact plus one right contact is two steps, not one stride.
2. Compare each numbered candidate time in the CSV and notebook plot with the
   corresponding foot lift/contact. Note false positives and missed steps.
3. Review ankle visibility and `NO POSE` intervals. Check framing before adjusting
   thresholds. A correct total can still hide offsetting misses and false positives.
4. Keep manual labels for several recordings with different speeds and distances.
   Evaluate changes on recordings other than those used for tuning.

The notebook includes pose/visibility coverage, event timelines, cumulative counts,
interval distributions, local cadence, a threshold-sensitivity grid, and optional
one-to-one matching against manual foot-lift timestamps (precision, recall, F1,
and timing errors). Each section explains its purpose and limits. Enter manual
labels for the chosen interval to enable accuracy evaluation; it is skipped by
default. Full landmark export and a saved manual-label scoring report are still
useful next additions. Synthetic tests below verify counting
behavior; they do not measure accuracy on people.

## Project structure

```text
.
├── pyproject.toml             # Package metadata, editable install, CLI entry point
├── requirements.txt           # Base workflow dependencies
├── requirements-pose.txt      # Optional live comparison backends
├── docs/MODELS.md             # Model purposes, sources, logs, comparison protocol
├── src/walk_analysis/
│   ├── __main__.py            # Command dispatcher
│   ├── paths.py               # Default input, model, and output locations
│   ├── tracking.py            # Offline MediaPipe extraction
│   ├── pose_models.py         # Live model catalog and adapters
│   ├── cadence.py             # Candidate detection and metrics
│   ├── analysis.py            # Analysis command
│   ├── debug.py               # Annotated video and candidate CSV export
│   ├── download_model.py      # All model/checkpoint downloads
│   ├── download_video.py      # YouTube MP4 download
│   └── environment.py         # Dependency and native-library checks
├── notebooks/                # Interactive exploration and evaluation
├── tests/                    # Automated regression tests
├── data/                     # Current input recording: walk.mp4 (local)
├── models/                   # Downloaded pose models and YOLOX companion (local)
├── outputs/                  # Previews, candidate CSVs, webcam text logs (local)
├── old videos/               # Archived experiments (local; see below)
└── .devcontainer/             # Reproducible development setup
```

The `src` layout keeps reusable code separate from notebooks, tests, and recordings.
`tracking.py` supplies offline MediaPipe extraction and `cadence.py` supplies
counting for the offline CLI and notebook. The live notebook uses the separate
`pose_models.py` adapters and does not calculate walking metrics.
Only debug export retains full poses in memory for rendering overlays. Longer
recordings may require a streaming export implementation.

The former root scripts have moved into the package; use the commands above instead
of `python analyze_video.py` or `python debug_video.py`. After updating an existing
checkout, run the editable-install command and restart the notebook kernel.

## Archived experiments: `old videos/`

This folder preserves earlier inputs and their generated results. The existing
`first video/` and `second video/` subfolders each contain:

- `walk.mp4`: the input used for that experiment.
- `walk_debug.mp4`: its annotated analysis preview.
- `step_candidates.csv`: candidate events from that analysis.

These files were preserved during the reorganization. They are historical snapshots:
counts and overlays may come from an earlier algorithm and need not match a new run.
They are not automatically processed by the notebook or commands. Paths with spaces
must be quoted in terminal commands, as shown above. Reanalyze into a new directory
under `outputs/` to keep the original archived results intact.

For future archives, use a descriptive subfolder per recording and include a small
notes file with source URL, manual count, date, settings, and package versions.
The current archives do not include those provenance notes. Recordings, models,
outputs, and archives are ignored by Git and Docker's build context, so keep a
separate backup if you need them after moving or recreating the workspace.

## Dependencies and checks

The container uses Python 3.12. `requirements.txt` specifies MediaPipe and the Python
dependencies. The Dockerfile installs native OpenGL/EGL/GLES, GLib, and audio
libraries. After changing dependencies or the Dockerfile, run
**Dev Containers: Rebuild Container**.

To install Python dependency changes into the current container:

```sh
python -m pip install --no-build-isolation -e .
python -m pip check
python -m walk_analysis check
python -m unittest discover -s tests -v
```

Tests cover alternating feet whose average cancels out, stationary signals,
visibility rejection, different sampling rates, tracking gaps, metric agreement,
and invalid inputs. A complete check also runs the notebook on a real recording
and opens the generated debug MP4. Package ranges are not a full environment lock;
record installed versions when comparing experiments across machines.

## Troubleshooting

- **Missing model:** run `python -m walk_analysis download-model`.
- **Missing video:** provide `data/walk.mp4` or run the downloader.
- **`fps` is not defined:** earlier cells have not run; restart the kernel and run all.
- **Unexpected keyword such as `left_visibility`:** the kernel has an old imported
  function; restart it and run all after changing Python files.
- **`libEGL.so.1` or `libGLESv2.so.2` missing:** rebuild the container. The Dockerfile
  includes `libegl1` and `libgles2`.
- **TensorFlow Lite INFO/feedback messages:** these can appear during successful
  inference. Inspect an actual traceback if a cell stops.
- **YouTube format/runtime errors:** update with
  `python -m pip install -U 'yt-dlp[default]' deno`, then retry. Authentication,
  availability, or site changes may still prevent a particular download.
- **No useful poses:** try a clearer full-body video. Metrics require at least
  three pose-detected frames; candidate counting may still reject all ankle samples.

## Camera access and live troubleshooting

The live notebook uses your Mac/PC browser to capture video; Docker does not need
direct USB passthrough. Use an external browser on the computer connected to the
camera, at localhost or a private HTTPS forwarded URL. The server processes frames
in memory; video is not recorded. Text logs contain timestamps and pose diagnostics.

- **Nothing at localhost:** forward port 8765 and run the notebook through step 4.
  Port forwarding alone does not start a server. Do not Run All; the cleanup cell
  stops the server immediately. Use VS Code's actual forwarded address if it maps
  to a different local port.
- **USB camera missing:** quit QuickTime/other camera apps, allow the browser in
  macOS System Settings → Privacy & Security → Camera, and allow the page's camera
  permission. Reload and click Enable camera to rebuild the device list.
- **No text log:** step 4 should print both “Server running” and “Detection text
  log”. If the latter is missing, reopen the updated notebook and restart the
  kernel. A header-only file means no successful frames were logged; check browser
  errors and whether the server started successfully.
- **Model disabled:** install its optional backend/download weights, rerun step 1
  and step 4, and reload the page. Installation is not automatic on container rebuild.
- **Skeleton disappears:** NO POSE means no accepted candidate; a partial skeleton
  can mean low joint scores or points outside the image. Keep the whole body and
  feet visible for the baseline. Lowering the drawing threshold only exposes weak
  predictions.
- **Port in use:** stop the other notebook server or choose and forward another
  PORT. Restarting the current kernel does not stop another kernel's server.

See [the model guide](docs/MODELS.md) for score meanings, timing, log fields,
checkpoint provenance, known package metadata limitations, and comparison results.

## Next steps

- Compare every model on identical recorded frames with manually checked joints.
- Label full-body, lower-body crop, empty-scene and occlusion intervals separately.
- Measure leg-joint correctness, tracking gaps and observed FPS alongside scores.
- Evaluate camera resolution, compression, distance and viewpoint consistently.
- Extend offline extraction to the live adapters before comparing step counts
  across families; validate candidate steps against manual labels.
- Add synchronized video and complete joint-coordinate export for later review.

## References

- [Model guide: all model sources and implementation details](docs/MODELS.md)
- [MediaPipe Pose Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker)
- [yt-dlp](https://github.com/yt-dlp/yt-dlp)
- [yt-dlp JavaScript runtime setup](https://github.com/yt-dlp/yt-dlp/wiki/EJS)
