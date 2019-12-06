#!/bin/bash

timestamp() {
  date +"%Y-%m-%d :: %H:%M:%S"
}

## default execution time definition
declare -A TIMES=( ["100"]="300" ["200"]="900" ["400"]="900" ["600"]="1800" ["800"]="3600" ["1000"]="3600" )

ipath="."
if [ "$1" == "" ] ; then
	echo "Missing instances path."
	exit 0
else
	ipath=$1
fi

base_folder=""
if [ "$2" == "" ] ; then
	echo "Missing result folder."
	exit 0
else
	base_folder=$2
fi

nt=""
if [ "$3" == "" ] ; then
	echo "Missing number of threads."
	exit 0
else
	nt=$3
fi

rep=""
if [ "$4" == "" ] ; then
	echo "Missing number of repetitions."
	exit 0
else
	rep=$4
fi

bp=""
if [ "$5" == "" ] ; then
	echo "Missing basic parameters"
	exit 0
else
	bp=$5
fi

par=""

res_folder=$base_folder"results"
sol_folder=$base_folder"solutions"
log_folder=$base_folder"logs"

for filename in $ipath*.txt
do
	## basename returns the last portion of a unix path
	fn=$(basename "$filename")
	## get the instance name only
	inst_name=$(echo $fn | cut -d'.' -f 1)

	## define the instance size in order to automatically select the maximum running time
	sz=$(echo $inst_name | cut -d"_" -f 2)
	if [ "$sz" == "$inst_name" ] ; then
		sz="100"
	else
		sz=$sz"00"
	fi
	
	time="${TIMES[$sz]}"
	
	for((r=0; r<$rep; r++ ))
	do
		## create corresponding seed folder if needed
		seed_folder="run_"$r
		mkdir -p $res_folder"/"$seed_folder"/"
		mkdir -p $sol_folder"/"$seed_folder"/"
		mkdir -p $log_folder"/"$seed_folder"/"
		res=$res_folder"/"$seed_folder"/"$inst_name".dat"
		sol=$sol_folder"/"$seed_folder"/"$inst_name".sol"
		log=$log_folder"/"$seed_folder"/"$inst_name".log"
		par=$par$filename","$res","$r",-f ll -s "$r" -t "$time" --log-file "$log" --solution "$sol" "$bp","
	done
done		
echo "Starting parallel runs: "$(timestamp)
echo $par | parallel --no-run-if-empty -j $nt -n 4 -d , ./target.sh
echo "Done with parallel runs: "$(timestamp)
#make -C ../ clean

echo "DONE"
