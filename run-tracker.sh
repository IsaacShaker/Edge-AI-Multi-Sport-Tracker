#!/bin/bash
# Wrapper script to run tracker with Qt warnings suppressed

# Activate virtual environment
source .venv/bin/activate

# Run with Qt warnings suppressed
# The 2> >(...) syntax redirects stderr through a filter, keeping stdout intact
python compute-vision/cv-kinematic-prose-tracker-v2.py "$@" 2> >(grep -v "QFontDatabase" | grep -v "XDG_SESSION_TYPE=wayland" | grep -v "Qt no longer ships fonts" | grep -v "fontconfig" | grep -v "dejavu-fonts" >&2)
