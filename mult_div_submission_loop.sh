#Examples:
# ./mult_div_submission_loop.sh 1 1 900 Uniform100m N0-Q100 params/params_search_n100_q40.csv
# ./mult_div_submission_loop.sh 1 1 900 OneCluster100m var-s2s-u2s params/params_var_s2s_u2s.csv

hpc=$1
if (( hpc )); then
    start=$2
    command=qsub
else
    start=$3
    command=bash
fi
stop=$3
world=$4
experiment=$5
param_file=$6
port_shift=$7 #default is 0, but can take a value to prevent gzserver of two separate experiments from overlapping
swarmsize=$8
simulation_folder=$PWD/results/$(date '+%Y-%m-%d')-$world-$experiment/r$swarmsize

for (( paramLine = start; paramLine <= stop; paramLine++)) do
    echo $paramLine # $(( $paramLine + $stop))
    $command hpc_qsub.sh $hpc $world $experiment $param_file $paramLine $port_shift $swarmsize $simulation_folder
done