#!/bin/bash

# Create input file with commands
cat > test_commands.txt << EOF
initialize
scheduler-start
EOF

# Run the program with commands
./build/sim < test_commands.txt