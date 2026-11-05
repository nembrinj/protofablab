FROM mcr.microsoft.com/devcontainers/python:1-3.12-bookworm

# Shared libraries used by MediaPipe, OpenCV, and audio dependencies.
# Use Debian sources; the base image's optional Yarn repo may have expired keys.
RUN apt-get -o Dir::Etc::sourcelist="sources.list.d/debian.sources" \
    -o Dir::Etc::sourceparts="-" update \
    && apt-get install -y --no-install-recommends \
    libgl1 \
    libegl1 \
    libgles2 \
    libglib2.0-0 \
    libportaudio2 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/project-requirements.txt
RUN python -m pip install --no-cache-dir -r /tmp/project-requirements.txt \
    && python -m pip check \
    && python -c "import mediapipe; from mediapipe.tasks.python.core.mediapipe_c_bindings import load_raw_library; load_raw_library(); print(mediapipe.__version__)"
