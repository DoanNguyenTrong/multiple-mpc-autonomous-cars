#!/bin/bash
# Script to run PID controller
cd ./build
chmod +x mpc
./mpc 1000 1000 1 50 50 250000 5000