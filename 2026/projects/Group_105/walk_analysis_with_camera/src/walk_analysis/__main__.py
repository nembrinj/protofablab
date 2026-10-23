"""Command dispatcher; run python -m walk_analysis --help."""
import argparse
import importlib
import sys

COMMANDS = {
    "analyze": "analysis", "debug": "debug", "download-video": "download_video",
    "download-model": "download_model", "check": "environment",
}


def main():
    parser = argparse.ArgumentParser(description="Analyze and inspect walking videos")
    parser.add_argument("command", choices=COMMANDS)
    args = parser.parse_args(sys.argv[1:2])
    module = importlib.import_module(f"walk_analysis.{COMMANDS[args.command]}")
    module.main(sys.argv[2:])


if __name__ == "__main__":
    main()
