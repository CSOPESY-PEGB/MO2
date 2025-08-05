#!/bin/bash

# Test script for Task 5
cd /Users/armaine/Documents/MO2

# Copy the config file
cp config_task5.txt config.txt

# Run the program with input commands
echo -e "initialize\nscheduler-test\n" | ./build/sim