# Pose model guide

This project compares eight pretrained checkpoints from five pose model families.
It aims to find useful hips, knees and ankles for later lower-body walking analysis.
The selection is a practical test set, not a verified “top five” popularity ranking.
No checkpoint has yet been validated as the best model for this camera or crop.

[Project setup](../README.md) · [Live notebook](../notebooks/02_live_webcam_pose.ipynb) · [Model catalog and adapters](../src/walk_analysis/pose_models.py) · [Downloader](../src/walk_analysis/download_model.py)

## What each model is useful for

The purposes below describe why to include each checkpoint in this experiment.
They are test hypotheses, not a measured ranking on our recordings.

| Browser name / CLI ID | Intended role in our tests | Output | Main tradeoff |
| --- | --- | --- | --- |
| MediaPipe Lite / `lite` | Lightweight baseline for one-person movement and foot tracking | 33 landmarks | Favor a small model; check whether fine leg movements are stable |
| MediaPipe Full / `full` | Middle option between Lite and Heavy | 33 landmarks | Test a larger landmark model without jumping straight to Heavy |
| MediaPipe Heavy / `heavy` | Largest MediaPipe option, to test whether extra model capacity helps difficult poses | 33 landmarks | More capacity does not guarantee recovery of cropped feet |
| YOLOv8n Pose / `yolov8n` | Compact person detection plus pose baseline, useful for comparison against MediaPipe | COCO 17 keypoints | Nano is the smallest size, not the most accurate checkpoint in its family |
| YOLO11n Pose / `yolo11n` | Another compact YOLO generation for the same hips/knees/ankles task | COCO 17 keypoints | Needs PyTorch; compare on identical frames rather than assuming newer is better |
| MoveNet Thunder / `movenet-thunder` | Single-person movement/fitness baseline using a lightweight inference runtime | COCO 17 keypoints | Thunder prioritizes quality over MoveNet Lightning; this adapter uses the full frame, without adaptive person cropping |
| RTMPose-m + YOLOX-m / `rtmpose-m` | Detector followed by pose estimation on each detected person crop | COCO 17 keypoints | Includes the cost and possible failures of a separate person detector |
| RTMO-m / `rtmo-m` | One-stage alternative to the RTMPose detector-plus-pose pipeline | COCO 17 keypoints | Different detection/pose tradeoff; medium checkpoint may be slower than nano options on this CPU |

MediaPipe's task is designed for body landmarks in images and video, including
33 full-body points. The three bundles share the task API. Our live adapter uses
only normalized 2D image landmarks; it does not export MediaPipe world coordinates.
[Official MediaPipe task documentation](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker)

The YOLO files are **pose checkpoints**, not generic object-detection weights.
They detect people and predict joints. The project uses nano (`n`) checkpoints
for a manageable CPU baseline. The installed Ultralytics package version and the
checkpoint architecture are separate: installing a newer package does not turn
YOLOv8n into YOLO11n. [YOLOv8 documentation](https://docs.ultralytics.com/models/yolov8/),
[YOLO11 documentation](https://docs.ultralytics.com/models/yolo11/)

MoveNet predicts 17 joints; Google describes Thunder as the quality-oriented
variant and Lightning as the latency-oriented variant. Only Thunder is integrated
here. [Official MoveNet tutorial and keypoint definitions](https://www.tensorflow.org/hub/tutorials/movenet)

RTMPose is a real-time pose framework from OpenMMLab/MMPose. Our selected body7
checkpoint is deployed with YOLOX-m through RTMLib; it is not the 133-point
whole-body variant. RTMO is a one-stage multi-person pose framework, also from
MMPose, run here through RTMLib. Upstream benchmark speeds use different hardware
and pipelines and are not predictions of our notebook's FPS.
[RTMPose paper](https://arxiv.org/abs/2303.07399),
[RTMO paper](https://arxiv.org/abs/2312.07526),
[RTMLib implementation and checkpoint catalog](https://github.com/Tau-J/rtmlib)

## Lower-body support and limits

Every option includes left/right hips, knees and ankles. Only the MediaPipe
checkpoints used here provide heels and foot tips. COCO-17 models cannot directly
measure toe position or foot orientation from ankle points alone.

| Joint | MediaPipe index, left / right | COCO-17 index, left / right |
| --- | --- | --- |
| Hip | 23 / 24 | 11 / 12 |
| Knee | 25 / 26 | 13 / 14 |
| Ankle | 27 / 28 | 15 / 16 |
| Heel | 29 / 30 | Not provided |
| Foot tip | 31 / 32 | Not provided |

Left/right means the person's anatomical side. Coordinates are normalized to the
submitted image: x increases rightward and y downward. They are not meters.
Estimated landmarks can fall outside [0, 1]; this does not make them visible.

These are full-body pretrained systems. A model supporting leg joints does not
establish that it can detect a person from legs alone. Cropping the head/torso,
cutting off feet, overlapping legs, motion blur, and small subjects can all affect
results. Larger weights do not solve missing visual evidence automatically.

For future knee/hip angle work, all options offer the needed 2D joints, but angles
still need validation against camera viewpoint and occlusion. This application
currently draws joints and logs scores; it does not compute joint angles, diagnose
gait, or validate step counts from the live stream.

## Where the weights come from

The table below matches `MODEL_SPECS` and `YOLOX_SPEC` in the source code.
The downloader does not train, fine-tune, quantize, or convert checkpoints.
MediaPipe bundles and YOLO weights are saved directly. For OpenMMLab ZIPs, it reads
the single ONNX member into a local file; other archive paths are not extracted.
MoveNet is a float16 TFLite bundle downloaded through Google's TF Hub endpoint.
Redirects may lead to a different hosting URL.

| CLI ID | Local file under `models/` | Upstream artifact |
| --- | --- | --- |
| `lite` | `pose_landmarker_lite.task` | [Download source](https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task) |
| `full` | `pose_landmarker_full.task` | [Download source](https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task) |
| `heavy` | `pose_landmarker_heavy.task` | [Download source](https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/1/pose_landmarker_heavy.task) |
| `yolov8n` | `yolov8n-pose.pt` | [Download source](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-pose.pt) |
| `yolo11n` | `yolo11n-pose.pt` | [Download source](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt) |
| `movenet-thunder` | `movenet_thunder.tflite` | [Download source](https://tfhub.dev/google/lite-model/movenet/singlepose/thunder/tflite/float16/4?lite-format=tflite) |
| `rtmpose-m` | `rtmpose_m.onnx` | [Download source](https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/rtmpose-m_simcc-body7_pt-body7_420e-256x192-e48f03d0_20230504.zip) |
| `rtmo-m` | `rtmo_m.onnx` | [Download source](https://download.openmmlab.com/mmpose/v1/projects/rtmo/onnx_sdk/rtmo-m_16xb16-600e_body7-640x640-39e78cc4_20231211.zip) |
| RTMPose person detector | `yolox_m.onnx` | [YOLOX-m source](https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/yolox_m_8xb8-300e_humanart-c2c7a14a.zip) |

MediaPipe URLs select `float16/1`. YOLO weights come from the Ultralytics assets
`v8.3.0` release (the release tag is not the installed Python package version).
The OpenMMLab URLs identify dated checkpoint/export archives. See the catalog
source for the full original filenames and input sizes.

Downloads require internet. Inference uses local weights. Models are ignored by
Git and the Docker build context; reinstall optional packages and download again
after rebuilding a container if their files are not preserved.

### Local artifact snapshot

The following SHA-256 values and sizes describe the files available when this
guide was updated on **2026-11-12**. They are local provenance records, not
publisher-signed checksums. The downloader performs basic bundle-format checks,
but does not enforce these hashes. Recheck after any redownload or model update.

| File | Size (MiB) | SHA-256 |
| --- | ---: | --- |
| `pose_landmarker_lite.task` | 5.51 | `59929e1d1ee95287735ddd833b19cf4ac46d29bc7afddbbf6753c459690d574a` |
| `pose_landmarker_full.task` | 8.96 | `5134a3aad27a58b93da0088d431f366da362b44e3ccfbe3462b3827a839011b1` |
| `pose_landmarker_heavy.task` | 29.24 | `64437af838a65d18e5ba7a0d39b465540069bc8aae8308de3e318aad31fcbc7b` |
| `yolov8n-pose.pt` | 6.52 | `c6fa93dd1ee4a2c18c900a45c1d864a1c6f7aba75d84f91648a30b7fb641d212` |
| `yolo11n-pose.pt` | 5.97 | `869e83fcdffdc7371fa4e34cd8e51c838cc729571d1635e5141e3075e9319dc0` |
| `movenet_thunder.tflite` | 12.00 | `41641538679ec79b07d4101e591dda47d098c09af29607674b2a40b8a3798dd3` |
| `rtmpose_m.onnx` | 51.81 | `5c0a4bf67953e6d2ac43ce15e77dc9d5d354ae18430a47d2c5963a7bc5683e3c` |
| `rtmo_m.onnx` | 85.13 | `750d872151ff4652f02c738efc3a547e112ce9b688fe920bc17f948e8c3afdac` |
| `yolox_m.onnx` | 96.70 | `3dea6513388889f0fff4b77bf7a26013600321b9eb9ceb0e9a400a82572f5f23` |

## Install and download

Run from the project root inside the devcontainer. The base environment and
editable package install are described in the main README.

```sh
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r requirements-pose.txt
python -m pip install --no-deps rtmlib==0.0.16
python -m walk_analysis download-model --model all
```

The CPU PyTorch index avoids downloading CUDA libraries. RTMLib is installed
without its dependencies to preserve this project's headless OpenCV; ONNX Runtime,
NumPy and the other required libraries are supplied separately. `pip check` can
report RTMLib's missing `opencv-python` distribution even though the headless
package supplies the working `cv2` import. Keep this known metadata limitation
separate from actual import/inference failures.

Optional backend specifications live in `requirements-pose.txt`. They are not
automatically installed by the base devcontainer build or by `pip install -e .`.
Restart the notebook kernel after installing or changing packages. Record actual
installed versions for each experiment; the requirements are not a full lockfile.

| Command argument | Downloads |
| --- | --- |
| No `--model` argument | MediaPipe Lite |
| `--model mediapipe` | Lite, Full, Heavy |
| `--model alternatives` | The five non-MediaPipe checkpoints, plus YOLOX-m |
| `--model all` | All eight pose choices, plus YOLOX-m |
| `--model yolov8n` (or another CLI ID above) | That model; RTMPose also downloads YOLOX-m |

`--output-dir DIR` changes the download folder. `--output FILE` is for a single
model; RTMPose still places its YOLOX companion alongside that file. The live
notebook discovers standard filenames in the project's `models/` directory, so
custom filenames/directories are not automatically discovered. Downloads overwrite
existing destination files on success; a failed format check leaves the previous
file intact. No resume, checksum verification, or automatic skip is implemented.

## Which workflow supports which models?

| Entry point | Supported models | Results |
| --- | --- | --- |
| `01_walk_experiment.ipynb` | MediaPipe task bundles; Lite by default | Experimental step candidates, cadence, plots, debug video |
| `analyze` and `debug` CLI commands | MediaPipe task bundles through `--model` | Walking metrics, candidate CSV, annotated video |
| `02_live_webcam_pose.ipynb` | All eight choices via `PoseDetector` | Browser skeleton, per-frame text diagnostics |

Passing `.pt`, `.onnx`, or `.tflite` files to the offline CLI's `--model` does not
enable the live adapters. Cross-family offline video evaluation remains future
work. The live notebook does not currently save video or calculate cadence.

## Browser workflow and model switching

1. Reopen the latest live notebook, restart the kernel, and run cells individually
   through step 4. Do not Run All: the final cell shuts down the server.
2. Forward port 8765 privately in VS Code and open the forwarded address in a
   browser running on the computer physically connected to the camera.
3. Allow camera access, click Enable camera, and choose the Logitech USB device.
4. Start the pose test and select a model. Missing packages/weights disable choices.
5. Switching loads a fresh detector, resets the browser coverage counter, and may
   pause for initialization. Frame numbering continues in the same server log.
6. Stop the camera, then run the notebook cleanup cell. Existing logs are retained.

The browser sends JPEG frames sequentially over HTTP to Python. The returned
points are drawn over the exact submitted frame. The previous completed image
remains visible during processing, avoiding a cleared skeleton between responses.
The stream is not a direct USB connection into Docker. Audio is not requested.
Use localhost or a private HTTPS forwarded page for browser camera permissions.
[Browser camera requirements](https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia)

## How scores, poses, and timing are defined

- MediaPipe uses VIDEO mode with one pose, detection confidence 0.5 and tracking
  confidence 0.5. It returns its first pose and native visibility scores.
- YOLO uses CPU inference, `imgsz=640`, person confidence threshold 0.25, then
  selects the person with the highest box confidence.
- MoveNet Thunder uses a centered square letterbox and its single-person output.
  Padding is inverted when mapping points back to the submitted image. It does
  not use the adaptive temporal cropping strategy from the upstream tutorial.
- RTMPose uses YOLOX-m at 640 × 640 followed by a 192 × 256 pose input. No detected
  boxes means NO POSE; the adapter avoids RTMLib's whole-image fallback.
- RTMO uses a 640 × 640 input with RTMLib's default detection/NMS settings.
  For RTMPose and RTMO, the retained candidate has the highest mean joint score.
- Every non-MediaPipe result also needs at least four keypoints scoring >= 0.2.
  This is our weak-prediction rejection heuristic, not a standardized pose metric.
- Only one person is returned. Identity can switch when several people appear.
  MediaPipe has temporal tracking; the other adapters run frame by frame.

MediaPipe visibility and other models' joint confidence are **different native
scores**, not accuracy probabilities calibrated across families. RTMPose scores
can exceed 1. The browser's `VISIBILITY` setting is a drawing threshold applied
to whichever native score the model returns. Lowering it reveals weak points;
it does not improve model accuracy or change the acceptance heuristic above.

COCO-17 points are mapped into the 33-slot display layout. Missing heels, toes,
and other unavailable points have score zero; the adapter does not invent them.
A detected pose can have missing/off-image ankles and is not proof of a usable
lower-body skeleton.

The log field `inference` measures the Python adapter call, including preprocessing,
person detection, pose inference and postprocessing. It excludes model loading,
JPEG decoding, network delay and browser drawing. The first call may include
backend warm-up. Browser round-trip time includes transport and response work.
Observed processed FPS also depends on the submission schedule and browser;
`TARGET_FPS=10` is a requested ceiling, not a guaranteed camera or inference rate.

## Text log reference

Each server start creates `outputs/webcam_detection_<UTC timestamp>.txt` and
prints the absolute path. The header is written before the server binds its port,
so an unsuccessful start can leave a header-only file. Each successful processed
frame is appended immediately and the file is closed after writing. NO POSE
frames are logged too. Failed requests/errors appear in browser status rather
than as detection rows.

| Field | Meaning |
| --- | --- |
| Timestamp | UTC server processing time, not camera exposure time |
| `model` | CLI ID of the selected checkpoint |
| `frame` | Sequence number of successfully processed requests since server start |
| `POSE DETECTED` / `NO POSE` | Adapter acceptance outcome, not accuracy |
| `inference` | Python adapter duration in milliseconds, as defined above |
| `score_kind` | `visibility` or `keypoint_confidence` |
| `ankle score L/R` | Native ankle scores for accepted poses |
| `ankle xy L/R` | Normalized 2D ankle coordinates |
| `leg scores` | Left/right hip and knee native scores |

Older logs can use `ankle visibility` and omit model/score-kind fields. `NO POSE` rows
omit joint scores rather than assigning them zero. Coordinates and scores are
rounded in text, so a logged 0.50 can be near either side of the drawing threshold.
There is no complete joint-coordinate export or synchronized video in this logger.

A lower-body quality review should report pose coverage, both-ankle coverage,
all-six-leg-joint coverage, out-of-frame estimates, tracking gaps, and visually
checked correctness. Separate missing poses from low-confidence joints. Confidence
threshold counts alone are not a cross-model accuracy benchmark.

## Experiment protocol and preliminary observations

For each model, record the same duration of full-body walking and then the intended
lower-body framing, with one person and fixed lighting/distance/resolution. Note
which intervals contain an empty scene, entry/exit, occlusion, and each crop.
Use the same recorded frames and manually checked joints for a fair later comparison.
Report processing time separately from camera throughput. Preserve input recordings,
model hashes, package versions and settings alongside results.

The following are uncontrolled local sessions from **2026-11-12**, not an accuracy
leaderboard. Framing/movement were not synchronized or labeled, and scores differ
by family. Threshold counts use rounded text values at 0.5. Empty-scene time can
reduce pose coverage without indicating a model error.

| Local log | Model | Frames | Accepted poses | Both ankle scores >= 0.5 | Median processing on pose-detected frames |
| --- | --- | ---: | ---: | ---: | ---: |
| `webcam_detection_20261112_144325_880159.txt` | Heavy | 850 | 402 (47.3%) | 4 (0.5% of all frames) | 51.2 ms |
| `webcam_detection_20261112_150331_441045.txt` | YOLOv8n | 582 | 307 (52.7%) | 103 (17.7% of all frames) | 89.2 ms |

Heavy had 244 accepted poses with at least one ankle coordinate outside the image;
its longest NO POSE run spanned about 31 seconds. YOLOv8n had a longest NO POSE run
of about 15.7 seconds, and only one frame where both hips, knees and ankles all
scored >= 0.5. These are signs to inspect framing and leg reliability, not evidence
that YOLOv8n is more accurate than Heavy. No controlled result for the other choices
has been established. Blank-image smoke-test logs are not camera experiments.

## Source and license references

- [MediaPipe task documentation and model links](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker)
- [Ultralytics pose task](https://docs.ultralytics.com/tasks/pose/) and [licensing information](https://www.ultralytics.com/license)
- [MoveNet official tutorial](https://www.tensorflow.org/hub/tutorials/movenet)
- [MMPose project](https://github.com/open-mmlab/mmpose), [RTMPose paper](https://arxiv.org/abs/2303.07399), [RTMO paper](https://arxiv.org/abs/2312.07526)
- [RTMLib repository and license](https://github.com/Tau-J/rtmlib)
- [LiteRT Python migration](https://ai.google.dev/edge/litert/migration)

Ultralytics provides AGPL and enterprise licensing options. RTMLib and MMPose
publish their code licenses in their repositories. A runtime's code license does
not by itself establish the terms for every pretrained weight or training dataset;
use each upstream artifact/model card when documenting redistribution or deployment.
