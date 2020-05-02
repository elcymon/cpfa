#USAGE
#./hpc_qsub world_name experiment row_shift prev_ID_end
#WHERE
#world_name is Uniform, OneCluster, TwoClusters, FourClusters, or HalfCluster

# Use current working directory and current modules
#$ -cwd -V

#$ -e /nobackup/scsoo/logs/errors
#$ -o /nobackup/scsoo/logs/outputs

# Request a full node (24 cores and 128 GB or 768 GB on ARC3)
# -l nodes=0.25
# -l node_type=24core-128G

# To request for x cores on a single machine, with around y memory per core
# -pe smp x -l h_vmem=yG

#memory?
#$ -l h_vmem=1G

#no of cores
#$ -pe smp 3

# Request Wallclock time of hh:mm:ss
#$ -l h_rt=2:0:0

#Iterations
#$ -t 1-30

#Iterations in batch of
#$ -tc 30


#e-mail
#$ -m a
#$ -M scsoo@leeds.ac.uk

#Run the job
#You can add cd to program directory to be sure
# environment variable SGE_TASK_ID varies based on range in -t option
#load singularity
echo $@
echo
hpc=$1 # true if working on hpc false otherwise

if (( $hpc==1 )); then
    module load singularity
    folder=/nobackup/scsoo/git_swarm_sim
    gzmode=gzserver
else
    folder=../swarm_sim
    JOB_ID=123
    SGE_TASK_ID=1
    gzmode=gazebo
    QUEUE=mypc
fi

#execute simulation
local_loc=$folder/local

#set python script input arguments
#name of the world to simulate on
world_name=$2
#experiment is used to know which parameter you are investigating
experiment=$3
param_file=$PWD/$4
#how many rows of parameters should be ignored use 0 if none
param_line=$5
port_shift=$6 #to prevent overlap with another gzserver of a different experiment submission
swarmsize=$7

simulation_folder=$8 #folder to save simulation results
simulation_prefix=_$(printf "%03d\n" $SGE_TASK_ID)"_"$JOB_ID"_"$(date '+%Y-%m-%d-%H-%M-%S')
#there should be no repetition of server port or else they will overwrite each other. Adding 1 just to be safe
port_number=$(( $SGE_TASK_ID + ( $port_shift + $param_line ) * 32 ))
echo world: $world_name, swarmsize: $swarmsize
echo param_file: $param_file
echo param_line: $param_line
echo simulation_folder: $simulation_folder
echo simulation_prefix: $simulation_prefix
echo port_number: $port_number
echo gzmode: $gzmode
mkdir -p $local_loc/$JOB_ID.$SGE_TASK_ID.$QUEUE $simulation_folder

singularity exec --bind $PWD:$PWD,$local_loc:/local $folder/20190708-libgazebo7-xenial.simg python3 hpc_start_simulation2.py $world_name $swarmsize $param_file $param_line $simulation_folder $simulation_prefix $port_number $gzmode 