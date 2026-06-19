#!/bin/bash 
#-------------------------------------------------------------- 
#   Script: build_swim_files.sh  
#   Author: Michael Benjamin   
#   LastEd: April 2023
#-------------------------------------------------------------- 
#  Part 1: Define a convenience function for producing terminal
#          debugging/status output depending on the verbosity.
#-------------------------------------------------------------- 
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }


#-------------------------------------------------------------- 
#  Part 2: Set Global variables
#-------------------------------------------------------------- 
OPA1="-138.0,-13.4:-89.3,-42.8:-68.3,-10.6:-90.4,-11.3"
OPA2="-161.7,-9.9:-85.2,-56.1:-52.2,-5.5:-86.9,-6.6"
OPA3="-185.4,-6.4:-81.1,-69.4:-36.1,-0.4:-83.4,-1.9"
OPA4="-215.0,-2.0:-76.0,-86.0:-16.0,6.0:-79.0,4.0"


gen_swimmers --poly=$OPA1 --swimmers=5  --sep=3 > athens_01.txt
gen_swimmers --poly=$OPA2 --swimmers=7  --sep=5 > athens_02.txt
gen_swimmers --poly=$OPA3 --swimmers=9  --sep=6 > athens_03.txt
gen_swimmers --poly=$OPA4 --swimmers=11 --sep=7 > athens_04.txt
