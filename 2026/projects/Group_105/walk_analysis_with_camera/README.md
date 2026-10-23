# Walking video experiment

An offline Python experiment that tracks a person's pose, estimates step candidates
and cadence, and exports a video showing exactly which movements were counted.
Use the notebook to explore results and the Python scripts to repeat an analysis.

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

## Command-line analysis

Run these commands from the project root:

```sh
python -m walk_analysis analyze data/walk.mp4
python -m walk_analysis debug
```

Both analysis and debug commands accept a video path and `--model PATH`.
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

**Model:** MediaPipe Pose Landmarker Lite, downloaded by the `download-model` command. It
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
├── requirements.txt           # Runtime dependency specifications
├── src/walk_analysis/
│   ├── __main__.py            # Command dispatcher
│   ├── paths.py               # Default input, model, and output locations
│   ├── tracking.py            # Shared metadata and pose extraction
│   ├── cadence.py             # Candidate detection and metrics
│   ├── analysis.py            # Analysis command
│   ├── debug.py               # Annotated video and candidate CSV export
│   ├── download_model.py      # Official model download
│   ├── download_video.py      # YouTube MP4 download
│   └── environment.py         # Dependency and native-library checks
├── notebooks/                # Interactive exploration and evaluation
├── tests/                    # Automated regression tests
├── data/                     # Current input recording: walk.mp4 (local)
├── models/                   # Downloaded pose model (local)
├── outputs/                  # Generated previews and CSVs (local)
├── old videos/               # Archived experiments (local; see below)
└── .devcontainer/             # Reproducible development setup
```

The `src` layout keeps reusable code separate from notebooks, tests, and recordings.
`tracking.py` is the single pose-extraction implementation; `cadence.py` is the
single counting implementation. The CLI and notebook call these same modules.
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

## Camera access

This workflow uses saved files. Docker Desktop on macOS does not automatically
forward the Mac webcam into a Linux container. Capture locally and copy the file,
or use a network stream in a future capture implementation. On Linux, direct
capture requires explicitly forwarding the camera device. Desktop preview windows
also need display access; the current outputs are notebook plots and MP4 files.

## Next steps

- Try the current workflow with video recorded from a camera instead of relying
  on YouTube videos.
- Record and test videos with different body framings, including the whole body
  in frame and only the upper or lower half of the body.
- Test other pose-detection models to determine which works best with videos
  showing only the lower body.

## References

- [MediaPipe Pose Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker)
- [yt-dlp](https://github.com/yt-dlp/yt-dlp)
- [yt-dlp JavaScript runtime setup](https://github.com/yt-dlp/yt-dlp/wiki/EJS)
