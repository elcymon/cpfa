# this shell script should load the simulation environment
hpc=$1

if ((hpc)); then
    module load singularity
    folder=/nobackup/scsoo
else
    folder=.
    JOB_ID=123
    SGE_TASK_ID=1
fi

#execute simulation
local_loc=$folder/local #gazebo needs location of /local folder
world_name=$2 #name of the world to simulate
experiment=$3 #experiment is used to know which parameters are varied/investigated
row_shift=$4 #how many rows of parameters should be ignored?
prev_ID_end=$5 #previous SGE_TASK_ID maximum value
line_number=$(($SGE_TASK_ID + $row_shift))

#there should be no repetition of server port or else they will overwrite each other. Adding 1 just to be safe
port_number=$(($SGE_TASK_ID + $prev_ID_end + 1))
echo world_name: $world_name, experiment: $experiment, row_shift: $row_shift, prev_ID_end: $prev_ID_end, line_number: $line_number, port_number: $port_number
mkdir -p $local_loc/$JOB_ID.$SGE_TASK_ID.24core-128G.q $folder/results

singularity exec --bind $folder/results:$PWD/results,$folder/results:$folder/swarm_sim*/results,$local_loc:/local \
../swarm_sim/gazebo-libgazebo7-xenial.simg \
python3 hpc_start_simulation2.py $world_name $experiment $line_number $port_number
