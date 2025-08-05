#!/bin/bash
cd /Users/armaine/Documents/MO2/build

# Start the simulator in background
./sim &
SIM_PID=$!

# Function to send command to sim
send_command() {
    echo "$1" > /proc/$SIM_PID/fd/0 2>/dev/null || true
}

sleep 1
send_command "initialize"
sleep 1
send_command "scheduler-start"
sleep 3
send_command "screen -ls"
sleep 2
send_command "screen -ls"
sleep 5
send_command "vmstat"
sleep 1
send_command "exit"

wait $SIM_PID