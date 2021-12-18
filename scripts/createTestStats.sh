#!/bin/bash

NUMCPUS=`grep ^proc /proc/cpuinfo | wc -l`
FIRST=`cat /proc/stat | awk '/^cpu / {print $5}'`
sleep 1
SECOND=`cat /proc/stat | awk '/^cpu / {print $5}'`
USED=`echo 2 k 100 $SECOND $FIRST - $NUMCPUS / - p | dc | tr -d '-'`     # Need to be installed dc
USED=`echo ${USED} | awk '{printf "%.1f", $0}'`                          # Converts .X to 0.X