#!/bin/sh
set -eu
test -d /home/user/submission
install -d -m 755 /home/user/practice
install -m 644 "$(dirname "$0")/payload/cameras.jsonl" /home/user/practice/cameras.jsonl
install -m 644 "$(dirname "$0")/payload/dispatch-example.png" /home/user/practice/dispatch-example.png
