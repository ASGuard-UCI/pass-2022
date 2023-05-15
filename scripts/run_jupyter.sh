#!/bin/bash

port=5555
if [ $# -eq 0 ]; then
  echo "No port provided, use default: 5555"
else
  port=$1
fi

jupyter lab --ip 0.0.0.0 --no-browser --allow-root --port ${port}
