#!/bin/bash
# Simple script to test MCM generation with OScar

# Default values (edit as needed)
OSCAR_BIN="./OScar"  # Path to the OScar binary
INTERFACE="randpkt"    # Network interface
VEHICLE_ID=1          # Vehicle ID
ENABLE_MCM="--enable-MCMs-tx"
MCM_PRIORITY="--MCMs-priority 3"

# Run OScar with MCM generation enabled
$OSCAR_BIN \
  --interface $INTERFACE \
  --vehicle-id $VEHICLE_ID \
  $ENABLE_MCM \
  $MCM_PRIORITY \
  "$@"
