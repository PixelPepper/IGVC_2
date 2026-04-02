#!/usr/bin/env bash
# Stop Gazebo Classic so the next launch can start gzserver on port 11345.
killall -q gzserver gzclient 2>/dev/null || true
sleep 0.5
killall -9 -q gzserver gzclient 2>/dev/null || true
sleep 0.5
