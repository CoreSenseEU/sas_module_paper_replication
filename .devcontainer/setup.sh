#!/usr/bin/env bash 
set -e
export PIP_BREAK_SYSTEM_PACKAGES=1
pip install rosbags pandas matplotlib Jinja2 scipy
git clone https://github.com/kas-lab/tactical_retreat_kb.git
pip install ./tactical_retreat_kb/