#!/usr/bin/env bash

#/home/sw3390/DMLIDE/DMLIDE ~/Desktop/beam_model_dml/b.dml --ne -t 1E-5 -r 5E-4
#/home/sw3390/DMLIDE/DMLIDE ~/Desktop/cube_dml/cube.dml --ne -t 1E-4 -r 5E-3

inputDML="$1"
trialParam="$2"
varyParam="$3"
echo
echo "Input:" $inputDML
echo "Trials:" $trialParam
echo "Vary:" $varyParam
echo

# Make SCENARIOS directory
inputPath=`dirname "$inputDML"`
outputDML=$inputPath/scenarios
if [ -d "$outputDML" ]; then
  rm -r $outputDML
fi
mkdir $outputDML

inputFile=`basename "$inputDML"`
inputExt="${inputFile##*.}"
inputFile="${inputFile%.*}"

# Iterate over parameters
for s in $inputPath/*.stl; do
  cp $s $outputDML
done
for ((i=4; i<= $#; i++)); do
  scenario=$outputDML/$varyParam\_${!i}
  mkdir $scenario
  python $PWD/scripts/generateDMLFiles.py $inputDML $varyParam ${!i} $outputDML
  for ((t=1; t<=$trialParam; t++)); do
    trial=$scenario/TRIAL_$t
    mkdir $trial

    success=0
    while [ $success == 0 ]; do
      success=1
      ./DMLIDE $outputDML/$inputFile\_${!i}.$inputExt --ne -t 1E-5 -r 5E-4 -d $trial

      # Check to make sure deflection is not NaN
      deflection=`csvtool col 3 $trial/optMetrics.csv`
      readarray -t defs <<<"$deflection"
      for d in "${defs[@]}"; do
        if [ "$d" == "nan" ]; then
          success=0
        fi
      done
    done
  done
done
