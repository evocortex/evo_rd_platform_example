#!/bin/bash

###############################################################
# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
# Further regulations can be found in LICENSE file.           #
###############################################################

LOG_FILE="can_init.log"

echo "[START   ] --> Script: "$0 | tee $LOG_FILE

/usr/bin/sudo modprobe can
/usr/bin/sudo modprobe can-raw
/usr/bin/sudo modprobe slcan

echo "[ DONE   ] --> modeprobe" | tee -a $LOG_FILE


for DEVICE_NAME in /dev/USBtin*
do
  
  IFACE_NAME=$(basename "$DEVICE_NAME")
  echo "[    ATTR] --> Sym.  device: "$DEVICE_NAME | tee -a $LOG_FILE
  echo "[    ATTR] --> Interface: "$IFACE_NAME | tee -a $LOG_FILE
  echo "[     ACK] --> "$(/usr/bin/sudo slcan_attach -f -s6 -o $DEVICE_NAME 2>&1)  | tee -a $LOG_FILE 
  echo "[ DONE   ] --> slcan_attach" | tee -a $LOG_FILE
  echo "[     ACK] --> " $(/usr/bin/sudo slcand $DEVICE_NAME $IFACE_NAME 2>&1)  | tee -a $LOG_FILE
  echo "[ DONE   ] --> slcand" | tee -a $LOG_FILE
  echo "[     ACK] --> " $(/usr/bin/sudo ip link set $IFACE_NAME up 2>&1) | tee -a $LOG_FILE
  echo "[ DONE   ] --> ip link" | tee -a $LOG_FILE

  #slcand -o -s6 -t hw -S 3000000 $DEVNAME
done

echo "[DONE    ] --> Script: "$0 | tee -a $LOG_FILE
echo "[ERRORS  ] --> "$? | tee -a $LOG_FILE

exit $?
