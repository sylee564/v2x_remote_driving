#!/usr/bin/env bash
set -e
TOPIC=${1:-/front/camera/image_raw}
HZ=$(rostopic hz -w 2 -m $TOPIC 2>/dev/null | awk '/average rate/ {print $3}')
if [[ -z "$HZ" || $(echo "$HZ < 5" | bc -l) -eq 1 ]]; then
  echo "LOW FPS on $TOPIC: $HZ"
  exit 1
fi
echo "OK $TOPIC: $HZ fps"
