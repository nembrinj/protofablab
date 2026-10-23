"""Download a selected YouTube recording as MP4."""
import argparse
from pathlib import Path
import shutil
import sys

import imageio_ffmpeg
from yt_dlp import YoutubeDL
from yt_dlp.utils import DownloadError

if __package__ in (None, ""):
    # VS Code's Run Python File executes this file outside its package context.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from walk_analysis.paths import VIDEO_PATH
else:
    from .paths import VIDEO_PATH

# Previous experiment recordings, kept for future reference:
# First video: https://www.youtube.com/watch?v=UYabdOlXJm0
# Second video: https://www.youtube.com/watch?v=iXwngf4M1qw
# Third video: https://www.youtube.com/watch?v=FZOBfFP8PVE
DEFAULT_URL = "https://www.youtube.com/watch?v=FZOBfFP8PVE"


def download_video(url: str, output: Path) -> None:
    if output.suffix.lower() != ".mp4":
        raise ValueError("The output path must end in .mp4")
    output.parent.mkdir(parents=True, exist_ok=True)
    options = {
        "format": "bv[ext=mp4]+ba[ext=m4a]/b[ext=mp4]/bv+ba/b",
        "outtmpl": str(output.with_suffix(".%(ext)s")),
        "merge_output_format": "mp4",
        "postprocessors": [{"key": "FFmpegVideoConvertor", "preferedformat": "mp4"}],
        "ffmpeg_location": shutil.which("ffmpeg") or imageio_ffmpeg.get_ffmpeg_exe(),
        "noplaylist": True,
    }

    with YoutubeDL(options) as ydl:
        ydl.download([url])

    print(f"Video ready for your notebook: {output}")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Download a YouTube video as MP4")
    parser.add_argument('url', nargs='?', default=DEFAULT_URL)
    parser.add_argument('--output', type=Path, default=VIDEO_PATH)
    args = parser.parse_args(argv)
    try:
        download_video(args.url, args.output)
    except DownloadError:
        raise SystemExit("Download failed. See the yt-dlp error above.") from None


if __name__ == "__main__":
    main()
