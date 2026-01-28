#!/bin/bash
set -e

REQ_FILE=${1:-requirements/robot-requirements.apt}

sudo apt update
grep -vE '^\s*#' "$REQ_FILE" | xargs sudo apt install -y
