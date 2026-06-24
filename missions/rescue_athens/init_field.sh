#!/bin/bash
#------------------------------------------------------------
#   Script: init_field.sh
#   Author: M.Benjamin
#   LastEd: March 2025
#------------------------------------------------------------
#  Part 1: A convenience function for producing terminal
#          debugging/status output depending on verbosity.
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }

#------------------------------------------------------------
#  Part 2: Set global variable default values
#------------------------------------------------------------
ME=`basename "$0"`
VEHICLE_AMT="1"
VERBOSE=""
RAND_VPOS="no"

# custom
RAND_SWIMMERS=""
GAME_FORMAT="r1"
SWIMMERS=15
UNREGERS=0
SWIM_FILE="athens_rand.txt"
OP1="-138.0,-13.4:-89.3,-42.8:-68.3,-10.6:-90.4,-11.3"
OP2="-161.7,-9.9:-85.2,-56.1:-52.2,-5.5:-86.9,-6.6"
OP3="-185.4,-6.4:-81.1,-69.4:-36.1,-0.4:-83.4,-1.9"
OP4="-215.0,-2.0:-76.0,-86.0:-16.0,6.0:-79.0,4.0"
SWIM_REGION=$OP4

#------------------------------------------------------------
#  Part 3: Check for and handle command-line arguments
#------------------------------------------------------------
for ARGI; do
    CMD_ARGS+=" ${ARGI}"
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
	echo "$ME [OPTIONS] [time_warp]                      "
	echo "                                               "
	echo "Options:                                       "
	echo "  --amt=N            Num vehicles to launch    "
	echo "  --verbose, -v      Verbose, confirm values   "
	echo "  --rand, -r         Rand vehicle positions    "
	echo "                                               "
	echo "Options (custom):                              "
	echo "  --rsl, -rsl        Rand swim locations       " 
	echo "  --format=<format>  Game format r1,r2,rs1,rs2 " 
	echo "                                               "
	echo "  --op1              Gen rand swimmers in op1  " 
	echo "  --op2              Gen rand swimmers in op2  " 
	echo "  --op3              Gen rand swimmers in op3  " 
	echo "  --op4              Gen rand swimmers in op4  " 
	echo "  --swim_file=<file> Set the swim file         " 
	echo "  --swimmers=<15>    Rand gen N reg swimmers   " 
	echo "  --unreg=<0>        Rand gen N unreg swimmers " 
	exit 0;
    elif [ "${ARGI:0:6}" = "--amt=" ]; then
        VEHICLE_AMT="${ARGI#--amt=*}"
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
	VERBOSE=$ARGI
    elif [ "${ARGI}" = "--rand" -o "${ARGI}" = "-r" ]; then
        RAND_VPOS="yes"

    elif [ "${ARGI:0:9}" = "--format=" ]; then
        GAME_FORMAT="${ARGI#--format=*}"

    elif [ "${ARGI}" = "--rsl" -o "${ARGI}" = "-rsl" ]; then
	RAND_SWIMMERS="true"
    elif [ "${ARGI}" = "-op1" -o "${ARGI}" = "--op1" ]; then
	SWIM_REGION=$OP1
	RAND_SWIMMERS="true"
    elif [ "${ARGI}" = "-op2" -o "${ARGI}" = "--op2" ]; then
	SWIM_REGION=$OP2
	RAND_SWIMMERS="true"
    elif [ "${ARGI}" = "-op3" -o "${ARGI}" = "--op3" ]; then
	SWIM_REGION=$OP3
	RAND_SWIMMERS="true"
    elif [ "${ARGI}" = "-op4" -o "${ARGI}" = "--op4" ]; then
	SWIM_REGION=$OP4
	RAND_SWIMMERS="true"
    elif [ "${ARGI:0:11}" = "--swimmers=" ]; then
        SWIMMERS="${ARGI#--swimmers=*}"
	RAND_SWIMMERS="true"
    elif [ "${ARGI:0:8}" = "--unreg=" ]; then
        UNREGERS="${ARGI#--unreg=*}"
	RAND_SWIMMERS="true"

    else 
	echo "$ME: Bad Arg: $ARGI. Exit Code 1."
	exit 1
    fi
done

#------------------------------------------------------------
#  Part 4: Source local coordinate grid if it exits
#------------------------------------------------------------

#------------------------------------------------------------
#  Part 5: Set starting positions, speeds, vnames, colors
#------------------------------------------------------------
vecho "Setting starting position, speeds, vnames, colors"

VPOS_CNT=0
if [ -f "vpositions.txt" ]; then
    VPOS_CNT=`wc -l vpositions.txt | awk '{print $1}'`
fi
echo "VPOS_CNT = $VPOS_CNT"
echo "VEHICLE_AMT = $VEHICLE_AMT"
if [ "${VPOS_CNT}" != "${VEHICLE_AMT}" ]; then
    rm -f vpositions.txt 
fi
# Always regenerate deterministic starts so launcher updates are not masked by
# stale vpositions.txt from a prior same-vehicle-count run.
if [ "${RAND_VPOS}" = "yes" ]; then
    pickpos --poly="$SWIM_REGION" --buffer=5 \
            --amt=$VEHICLE_AMT --hdg="0,359,0" > vpositions.txt
else
    {
        echo "-40,-30"
        echo "-44,-36"
        echo "-48,-42"
        echo "-51,-47.5"
    } | head -n "$VEHICLE_AMT" > vpositions.txt
fi

# generate randomly placed swimmers
if [ "${RAND_SWIMMERS}" != "" ]; then
    gen_swimmers --poly=$SWIM_REGION --swimmers=$SWIMMERS   \
                 --unreg=$UNREGERS --sep=7 > $SWIM_FILE
fi

# Set the speeds and names
pickpos --amt=$VEHICLE_AMT --spd=1.2:1.2 > vspeeds.txt 
pickpos --amt=$VEHICLE_AMT --vnames  > vnames.txt

# Handle the chosen game format
if [ "${GAME_FORMAT}" = "r2" ]; then
    echo -e "rescue\nrescue" > vroles.txt
    echo -e "abe\nabe"       > vmates.txt
    echo -e "yellow\nred"    > vcolors.txt
elif [ "${GAME_FORMAT}" = "rs1" ]; then
    echo -e "rescue\nscout" > vroles.txt
    echo -e "ben\nabe"      > vmates.txt   # abe→ben (ξέρει τον scout του), ben→abe
    echo -e "blue\nblue"    > vcolors.txt
elif [ "${GAME_FORMAT}" = "rs2" ]; then
    echo -e "rescue\nrescue\nscout\nscout" > vroles.txt
    echo -e "cal\ndeb\nabe\nben"           > vmates.txt   # abe→cal, ben→deb (κάθε rescue ξέρει τον scout της)
    echo -e "green\nblue\ngreen\nblue"     > vcolors.txt
else # format=r1
    echo "rescue" > vroles.txt
    echo "abe"    > vmates.txt
    echo "yellow" > vcolors.txt
fi


#------------------------------------------------------------
#  Part 6: Set other aspects of the field, e.g., obstacles
#------------------------------------------------------------
#pickpos --amt=$VEHICLE_AMT --lfile=rescue_apps.txt > vapps.txt
pickpos --amt=2 --lfile=rescue_apps.txt > vapps.txt

cat vapps.txt vapps.txt >> new.txt
mv new.txt vapps.txt

#------------------------------------------------------------
#  Part 7: If verbose, show file contents
#------------------------------------------------------------
if [ "${VERBOSE}" != "" ]; then
    echo "--------------------------------------"
    echo "CMD_ARG       = $CMD_ARGS"
    echo "--------------------------------------"
    echo "VEHICLE_AMT   = $VEHICLE_AMT"
    echo "RAND_VPOS     = $RAND_VPOS"
    echo "--------------------------------------"
    echo "RAND_SWIMMERS = $RAND_SWIMMERS"
    echo "SWIM_FILE     = $RAND_SWIMMERS"
    echo "SWIMMERS      = $SWIMMERS"
    echo "UNREGERS      = $UNREGERS"
    echo "--------------------------------------(pos/spd)"
    echo "vpositions.txt:"; cat  vpositions.txt
    echo "vspeeds.txt:";    cat  vspeeds.txt
    echo "--------------------------------------(vprops)"
    echo "vnames.txt:";     cat  vnames.txt
    echo "vcolors.txt:";    cat  vcolors.txt
    echo "--------------------------------------(vapps)"
    echo "vapps.txt:";      cat  vapps.txt
    echo -n "Hit any key to continue"
    read ANSWER
fi

exit 0
