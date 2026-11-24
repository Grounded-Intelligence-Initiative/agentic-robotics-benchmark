#!/usr/bin/env bash
# The entry point the grader starts: `bash run.sh`, working directory = the submission folder.
# stdout is the wire: print nothing on it but READY and the replies. Diagnostics go to stderr.
exec /opt/venv/bin/python3 -B -s controller.py
