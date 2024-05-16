#!/bin/bash

FILENAME=$1
DEVICE=$2
BAUDRATE=$3

echo "Replay started!"
python3 serialreplay_core.py --mode r --file ${FILENAME} | socat - PTY,link=${DEVICE},raw,b${BAUDRATE}