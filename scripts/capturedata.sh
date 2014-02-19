#!/bin/bash
key=$1
cd Data/${key}_player/ || exit 1

player ../player_worlds/$key.cfg &
pid=$!
sleep 2
../../bin/captureplayerdata
kill -2 $pid
