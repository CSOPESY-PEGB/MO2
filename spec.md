This document outlines the requirements and specifications for the CSOPESY Major Output: Process Scheduler and CLI.

CSOPESY Major Output: Process Scheduler and CLI

By group

Created By: Neil Patrick Del Gallego, PhD

Updated as of: May 7, 2025

[100 pts] General Instructions: The first part of your emulator is the process multiplexer and your command-line interpreter (CLI).

(The original document includes screenshots of a nano editor showing a pytest.sh script and a PowerShell window. The pytest.sh content is relevant for understanding SLURM-like commands. The PowerShell content demonstrates basic CLI interaction.)

Relevant content from pytest.sh (SLURM commands example):

Generated bash
#!/bin/bash
##This is called the SHEBANG. This tells the shell to interpret and run the SLURM
##script using the bash (Bourne-again shell) shell.

##SLURM Commands using #SBATCH --[options]

#SBATCH --job-name demoTest      ##give the job a name
#SBATCH --output demo.out        ##creates a file to store the job output
#SBATCH --mail-user searsg@uhd.edu ##sends email to user's email for start/complete of job
#SBATCH --mail-type BEGIN        ##creates an event type to be mailed to user when event occurs
#SBATCH --mail-type END          ##creates an event type to be mailed to user when event occurs
#SBATCH --error test.e.txt       ##creates a file to store any error output

##Load the modules needed for the job to run
module load python3

##List the executables commands (or a string of them)
##Example for code written in Python
python3 pytest.py 90

Shell Reference

Please refer to a general Linux/Windows powershell/Windows command line. This serves as a strong reference for the design of your command-line interface.

For the process multiplexer, refer to the Linux “screen” command on its behavior: https://www.geeksforgeeks.org/screen-command-in-linux-with-examples/

Checklist of Requirements

Your system must have ALL the following features implemented properly.

Requirement	Description
Main menu console	Console Mockup:<br>text<br> CSOPESY<br><br>Welcome to CSOPESY Emulator!<br><br>Developers:<br>Del Gallego, Neil Patrick<br><br>Last updated: 01-18-2024<br><br>root:\><br><br>A main menu console for recognizing the following commands:<br>- "initialize": Initialize the processor configuration of the application. This must be called before any other command could be recognized, aside from "exit".<br>- "exit": Terminates the console.<br>- "screen": See additional details.<br>- "scheduler-start" (formerly scheduler-test): Continuously generates a batch of dummy processes for the CPU scheduler. Each process is accessible via the “screen” command.<br>- "scheduler-stop": Stops generating dummy processes.<br>- "report-util": For generating CPU utilization report. See additional details.
“screen” command support	From the main menu, the user can perform the following:<br>- Create a new process via “screen -s <process name>" command.<br>- Lists all running processes via “screen -ls” command.
Barebones process instructions	Support basic process instructions, akin to programming language instructions:<br>- PRINT(msg) – display an output “msg” to the console. The output can only be seen when the user is inside its attached screen.<br> - The “msg” can print 1 variable, “var.” E.g. PRINT(“Value from: " +x)<br>- DECLARE(var, value) – declares a uint16 with variable name “var”, and a default “value”.<br>- ADD (var1, var2/value, var3/value) – performs an addition operation: var1 = var2/value + var3/value<br> - var1, var2, var3 are variables. Variables are automatically declared with a value of 0 if they have not yet been declared beforehand. Can also add a uint16 value.<br>- SUBTRACT (var1, var2/value, var3/value) – performs a subtraction operation: var1 = var2/value - var3/value<br>- SLEEP (X) – sleeps the current process for X (uint8) CPU ticks and relinquishes the CPU.<br>- FOR([instructions], repeats) – performs a for-loop, given a set/array of instructions. Can be nested.<br>NOTES<br>- Process instructions are pre-determined and not typed by the user. E.g., randomized via scheduler-start command.<br>- Variables are stored in memory and will not be released until the process finishes.<br>- uint16 variables are clamped between (0, max(uint16)).<br>- Unless specified in the test case, the “msg” in the PRINT function should always be “Hello world from <process_name>!”<br>- For loops can be nested up to 3 times.
Generation of CPU utilization report	The console should be able to generate a utilization report whenever the "report-util" command is entered.
Configuration setting	The "initialize” commands should read from a “config.txt" file, the parameters for your CPU scheduler and process attributes.
The "screen" command specifications

The "screen" command emulates the screen multiplexer of Linux OS. Below is a CLI mockup of the screen command:

Generated text
Process name: screen_01
ID: 1
Logs:
(08/06/2024 09:15:22AM) Core:0 "Hello world from screen_01!"
(08/06/2024 09:32:09AM) Core:1 "Hello world from screen_01!"

Current instruction line: 153
Lines of code: 1240

root:\> process-smi

Process name: screen_01
ID: 1
Logs:
(08/06/2024 09:15:22AM) Core:0 "Hello world from screen_01!"
(08/06/2024 09:15:28AM) Core:0 "Hello world from screen_01!"
(08/06/2024 09:32:09AM) Core:1 "Hello world from screen_01!"
(08/06/2024 09:33:12AM) Core:1 "Hello world from screen_01!"

Current instruction line: 769
Lines of code: 1240

root:\> process-smi

Process name: screen_01
ID: 1
Logs:
(08/06/2024 09:15:22AM) Core:0 "Hello world from screen_01!"
(08/06/2024 09:15:28AM) Core:0 "Hello world from screen_01!"
(08/06/2024 09:15:50AM) Core:0 "Hello world from screen_01!"
(08/06/2024 09:32:09AM) Core:1 "Hello world from screen_01!"
(08/06/2024 09:33:10AM) Core:1 "Hello world from screen_01!"
(08/06/2024 09:33:12AM) Core:1 "Hello world from screen_01!"

Finished!

root:\>
IGNORE_WHEN_COPYING_START
content_copy
download
Use code with caution.
Text
IGNORE_WHEN_COPYING_END

When the user types “screen -s <process_name>" from the main menu console, the console will clear its contents and “move” to the process screen (lines 162 onwards). From there, the user can type the following:

"process-smi": Prints a simple information about the process (lines 9 – 13). The process contains dummy instructions that the CPU executes in the background. Whenever the user types “process-smi", it provides the updated details and accompanying logs from the print instructions. (e.g., lines 162 – 170). If the process has finished, simply print "Finished!" after the process name, ID, and logs have been printed (e.g., lines 17 – 20).

"exit": Returns the user to the main menu.

The range of instruction length per process can be set through the “config.txt.” Instruction types are randomized.

At any given time, any process can finish its execution. If this happens, the user can no longer access the screen after exiting.

The user can access the screen anytime by typing “screen -r <process name>” in the main menu. If the process name is not found/finished execution, the console prints “Process <process name> not found."

Note that to debug/validating the correctness of your program, all finished and currently running processes must be reported in the “report-util” command.

The "scheduler-start" and "scheduler-stop" commands

To facilitate and stress-test the capabilities of your console, we should provide support for generating a batch of dummy processes.

“scheduler-start” – The behavior is as follows: Every X CPU ticks, a new process is generated and put into the ready queue for your CPU scheduler. This frequency can be set in the “config.txt." As long as CPU cores are available, each process can be executed and be accessible via the “screen” command.

"scheduler-stop” – Stops generating dummy processes.

These commands are only accessible in the main menu console.

You must generate human-readable process names for the processes generated by the “scheduler-test" command to conveniently access them using the “screen -s <process name>” command described earlier. E.g.: p01, p02, ..., p1240.

The “screen -ls” and “report-util" commands

These commands should be similar. The only difference is that “report-util" saves this into a text file – "csopesy-log.txt." See sample mockup:

Generated text
CSOPESY

Welcome to CSOPESY Emulator!

Developers:
Del Gallego, Neil Patrick

Last updated: 01-18-2024

root:\> screen -ls
CPU utilization: 100%
Cores used: 4
Cores available: 0

Running processes:
process05   (01/18/2024 09:15:22AM)   Core: 0   1235 / 5876
process06   (01/18/2024 09:17:22AM)   Core: 1   3 / 5876
process07   (01/18/2024 09:17:45AM)   Core: 2   9 / 1000
process08   (01/18/2024 09:18:58AM)   Core: 3   12 / 80

Finished processes:
process01   (01/18/2024 09:00:21AM)   Finished   5876 / 5876
process02   (01/18/2024 09:00:22AM)   Finished   5876 / 5876
process03   (01/18/2024 09:00:42AM)   Finished   1000 / 1000
process04   (01/18/2024 09:00:53AM)   Finished   80 / 80

root:\> report-util
root:\> Report generated at C:/csopesy-log.txt!
IGNORE_WHEN_COPYING_START
content_copy
download
Use code with caution.
Text
IGNORE_WHEN_COPYING_END

The “screen-ls" commands should list the CPU utilization, cores used, and cores available, as well as print a summary of the running and finished processes (lines 38 – 54). The “report-util" command saves the same info in the csopesy-log.txt file.

The scheduler

Your CPU scheduler is real-time and will continuously schedule processes as long as your console is alive. The scheduler algorithm will be set through the “initialize” command and through the “config.txt" file.

The CPU ticks

For simplicity, assume that the CPU tick is an integer counter that tallies the number of frame passes. See pseudocode below:

Generated c
int main() {
    int cpuCycles = 0;
    while (<OS is running>) {
        cpuCycles++;
    }
}
IGNORE_WHEN_COPYING_START
content_copy
download
Use code with caution.
C
IGNORE_WHEN_COPYING_END
The config.txt file and “initialize” command

The user must first run the “initialize” command. No other commands should be recognized if the user hasn't typed this first. Once entered, it will read the “config.txt" file, which is space-separated in format, containing the following parameters.

Parameter	Description
num-cpu	Number of CPUs available. The range is [1, 128].
scheduler	The scheduler algorithm: “fcfs” or “rr”.
quantum-cycles	The time slice is given for each processor if a round-robin scheduler is used. Has no effect on other schedulers. The range is [1, 2^32].
batch-process-freq	The frequency of generating processes in the “scheduler-start" command in CPU cycles. The range is [1, 2^32]. If one, a new process is generated at the end of each CPU cycle.
min-ins	The minimum instructions/command per process. The range is [1, 2^32].
max-ins	The maximum instructions/command per process. The range is [1, 2^32].
delays-per-exec	Delay before executing the next instruction in CPU cycles. The delay is a “busy-waiting” scheme wherein the process remains in the CPU. The range is [0, 2^32]. If zero, each instruction is executed per CPU cycle.

The default parameters and sample "config.txt" can be seen below:

Generated text
num-cpu 4
scheduler "rr"
quantum-cycles 5
batch-process-freq 1
min-ins 1000
max-ins 2000
delays-per-exec 0
IGNORE_WHEN_COPYING_START
content_copy
download
Use code with caution.
Text
IGNORE_WHEN_COPYING_END
ASSESSMENT METHOD

Your CLI emulator will be assessed through a black box quiz system in a time-pressure format. This is to minimize drastic changes or “hacking" your CLI to ensure the test cases are met. You should only modify the parameters and no longer recompile the CLI when taking the quiz.

Test cases, parameters, and instructions are provided per question, wherein you must submit a video file (.MP4), demonstrating your CLI. Some questions will require submitting PowerPoint presentations, such as cases explaining the details of your implementation.

IMPORTANT DATES

See AnimoSpace for specific dates.

Week 7	Mockup test case and quiz
Week 8	Actual test case and quiz
Submission Details

Aside from video files for the quiz, you need to prepare some of the requirements in advance, such as:

SOURCE - Contains your source code. Add a README.txt with your name and instructions on running your program. Also, indicate the entry class file where the main function is located. An alternative can be a GitHub link.

PPT – A technical report of your system containing:

Command recognition

Console UI implementation

Command interpreter implementation

Process representation

Scheduler implementation

Grading Scheme

You are to provide evidence for each test case, recorded through video. Each test case will have some points allocated. The test cases will be graded as follows:

Robustness	No points	Partial points	Full points
	The CLI did not pass the test case. NO WORKAROUND is available to produce the expected output.	The CLI did not pass the test case. A workaround is available to produce the expected output.	The CLI passed the test case using varying inputs and produced the expected output.
