#
# lasinfo.py
#
# (c) 2012, Martin Isenburg
# LASSO - rapid tools to catch reality
#
# uses lasinfo.exe to 
#
# The LiDAR input can be in LAS/LAZ/BIN/TXT/SHP/ASC... format.
#
# for licensing details see http://rapidlasso.com/download/LICENSE.txt
#

import sys, os, arcgisscripting, subprocess

def check_output(command,console):
    if console == True:
        process = subprocess.Popen(command)
    else:
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    output,error = process.communicate()
    returncode = process.poll()
    return returncode,output 

### create the geoprocessor object
gp = arcgisscripting.create(9.3)

### report that something is happening
gp.AddMessage("Starting lasinfo ...")

### get number of arguments
argc = len(sys.argv)

### report arguments (for debug)
#gp.AddMessage("Arguments:")
#for i in range(0, argc):
#    gp.AddMessage("[" + str(i) + "]" + sys.argv[i])

### get the path to the LAStools binaries
lastools_path = os.path.dirname(os.path.dirname(os.path.dirname(sys.argv[0])))+"\\bin"

### check if path exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find .\lastools\bin at " + lastools_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lastools_path + " ...")

### create the full path to the lasinfo executable
lasinfo_path = lastools_path+"\\lasinfo.exe"

### check if executable exists
if os.path.exists(lastools_path) == False:
    gp.AddMessage("Cannot find lasinfo.exe at " + lasinfo_path)
    sys.exit(1)
else:
    gp.AddMessage("Found " + lasinfo_path + " ...")

### create the command string for lasinfo.exe
command = [lasinfo_path]

### maybe use '-verbose' option
if sys.argv[argc-1] == "true":
    command.append("-v")

### add first input LiDAR
command.append("-i")
command.append(sys.argv[1])

### maybe user-specified where output goes
if sys.argv[2] != "#":
    command.append("-o")
    command.append(sys.argv[2])
elif sys.argv[3] == "none":
    command.append("-quiet")
elif sys.argv[3] == "stdout":
    command.append("-stdout")
elif sys.argv[3] == "*_info.txt":
    command.append("-otxt")

### maybe do not report the LAS header
if sys.argv[4] == "true":
    command.append("-nh")

### maybe do not report the VLRs
if sys.argv[5] == "true":
    command.append("-nv")

### maybe do not parse the points
if sys.argv[6] == "true":
    command.append("-nc")

### maybe do not report the min max values
if sys.argv[7] == "true":
    command.append("-nmm")

### maybe compute the point density
if sys.argv[8] == "true":
    command.append("-cd")

### maybe report progress 
if sys.argv[9] == "true":
    command.append("-progress")
    command.append("1000000")

### maybe repair the counters
if sys.argv[10] == "true":
    command.append("-repair_counters")

### maybe repair the bounding box
if sys.argv[11] == "true":
    command.append("-repair_bb")

### report command string
gp.AddMessage("LAStools command line:")
command_length = len(command)
command_string = str(command[0])
for i in range(1, command_length):
    command_string = command_string + " " + str(command[i])
gp.AddMessage(command_string)

### run command
returncode,output = check_output(command, False)

### report output of lasinfo
gp.AddMessage(str(output))

### check return code
if returncode != 0:
    gp.AddMessage("Error. lasinfo failed.")
    sys.exit(1)

### report happy end
gp.AddMessage("Success. lasinfo done.")
