#!/bin/bash

FILENAME=$1
DEVICE=$2
BAUDRATE=$3

echo "Recording started! Terminate with Ctrl+C..."
python3 serialreplay_core.py --mode l --file ${FILENAME} --device ${DEVICE} --baudrate ${BAUDRATE}