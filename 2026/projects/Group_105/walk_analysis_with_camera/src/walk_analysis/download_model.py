"""Download pose models: MediaPipe, YOLO Pose, MoveNet, RTMPose, and RTMO."""
import argparse
from io import BytesIO
from pathlib import Path
from urllib.request import urlopen
from zipfile import ZipFile

from .paths import MODEL_PATH
from .pose_models import MODEL_SPECS, MP_NAMES, YOLOX_SPEC

MODEL_URLS = {name: spec['url'] for name, spec in MODEL_SPECS.items()}
MODEL_URL = MODEL_URLS['lite']


def _download(spec, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    print(f"Downloading {destination.name}…", flush=True)
    with urlopen(spec['url'], timeout=180) as response:
        content = response.read()
    if '.zip' in spec['url']:
        with ZipFile(BytesIO(content)) as archive:
            names = [n for n in archive.namelist() if n.endswith('.onnx')]
            if len(names) != 1:
                raise ValueError(f"Expected one ONNX file, found {names}")
            content = archive.read(names[0])  # Do not extract arbitrary archive paths.
    if destination.suffix == '.tflite' and content[4:8] != b'TFL3':
        raise ValueError('Server returned an invalid TFLite model (possibly an HTML download page)')
    if destination.suffix in ('.pt', '.task') and not content.startswith(b'PK'):
        raise ValueError('Server returned an invalid model bundle')
    temporary = destination.with_suffix(destination.suffix + '.tmp')
    temporary.write_bytes(content)
    temporary.replace(destination)
    print(f"Saved {destination}", flush=True)


def download_model(destination: Path, model: str = 'lite') -> None:
    _download(MODEL_SPECS[model], destination)
    if model == 'rtmpose-m':
        _download(YOLOX_SPEC, destination.parent / YOLOX_SPEC['file'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=[*MODEL_SPECS, 'mediapipe', 'alternatives', 'all'], default='lite')
    parser.add_argument('--output', type=Path, help='Output file for one model')
    parser.add_argument('--output-dir', type=Path, default=MODEL_PATH.parent)
    args = parser.parse_args(argv)
    groups = dict(mediapipe=MP_NAMES, alternatives=[n for n in MODEL_SPECS if n not in MP_NAMES], all=MODEL_SPECS)
    if args.model in groups and args.output:
        parser.error('--output requires one model; use --output-dir for groups')
    for name in groups.get(args.model, [args.model]):
        destination = args.output or args.output_dir / MODEL_SPECS[name]['file']
        download_model(destination, name)


if __name__ == '__main__':
    main()
