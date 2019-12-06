#!/bin/bash

timestamp() {
  date +"%Y-%m-%d :: %H:%M:%S"
}

filename="."
if [ "$1" == "" ] ; then
	echo "Missing input file."
	exit 0
else
	filename=$1
fi

res_file=""
if [ "$2" == "" ] ; then
	echo "Missing result file."
	exit 0
else
	res_file=$2
fi

rep=""
if [ "$3" == "" ] ; then
	echo "Missing repetition number."
	exit 0
else
	rep=$3
fi

par=""
if [ "$4" == "" ] ; then
	echo "Missing general parameters."
	exit 0
else
	par=$4
fi

## basename returns the last portion of a unix path
fn=$(basename "$filename") 
## get the instance name only
ni=$(echo $fn | cut -d'.' -f 1)
## define the instance size
sz=$(echo $ni | cut -d"_" -f 2)
if [ "$sz" == "$ni" ] ; then
	sz="100"
else
	sz=$sz"00"
fi

itype="" ##instance type: R, C, RC
itw="" ##time window type: (1), (2)
offtw=2 ##offset for tw test
if [[ (${ni:1:1} = "C") || (${ni:1:1} = "c") ]] ; then
	itype="C"
elif [[ (${ni:1:2} = "RC") || (${ni:1:2} = "rc") ]] ; then
	itype="RC"
	offtw=3
else
	itype="R"
fi

if [[ ${ni:$offtw:1} = "1" ]] ; then
	itw="1"
else
	itw="2"
fi

#echo $filename" | "$res_file" | "$par
#timestamp
par=$par" --print table"
echo "Running "$rep" - "$ni" | Started: "$(timestamp)
out_main="$(../code/main $par < $filename)"
echo -e $ni";"$sz";"$itype";"$itw";"$rep";\n$out_main" >> $res_file
echo "Done    "$rep" - "$ni" | Finished: "$(timestamp)
#done
# echo -e "done" 
echo "----------"


