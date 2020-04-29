import subprocess
import sys
import os.path
from concurrent.futures import ThreadPoolExecutor

root_dir = '$PWD/sources/models/w_swarm/world_db/'
world_db = {
	'OneCluster':'20180208_w_swarm1_circular_one_region_cluster.world',
	'TwoClusters':'20180208_w_swarm1_circular_two_region_cluster.world',
	'FourClusters':'20180209_w_swarm1_circular_four_clusters.world',
	'HalfCluster':'20180209_w_swarm1_circular_half_cluster_half_uniform.world',
	'Uniform':'20180209_w_swarm1_circular_uniform_litter.world',
	'OneCluster100m':'20180208_w_swarm1_circular_one_region_cluster_100m.world',
	'TwoClusters100m':'20180208_w_swarm1_circular_two_region_cluster_100m.world',
	'FourClusters100m':'20180209_w_swarm1_circular_four_clusters_100m.world',
	'HalfCluster100m':'20180209_w_swarm1_circular_half_cluster_half_uniform_100m.world',
	'Uniform100m':'20180209_w_swarm1_circular_uniform_litter_100m.world',
	'OneCluster28m':'20180208_w_swarm1_circular_one_region_cluster_28m.world',
	
}

def start_simulation(world_name, swarmsize, params_file, params_line,
		simulation_folder, simulation_prefix, port_number, gzmode):
	port_number = int(port_number) + 11345
	
	print(('simulation_folder: {}\nparams_line: {}\nport_number: {}'
			'\nsimulation_prefix: {}'.format(simulation_folder,
			params_line,port_number,simulation_prefix)))
	copy_wp=None
	copy_ss=None
	print('Checking control plugins')

	if os.path.isfile('compiled_plugins/libwp_swarm.so'):
		copy_wp = 0
	else:
		copy_wp = 1

	if os.path.isfile('compiled_plugins/start_simulation'):
		copy_ss = 0
	else:
		copy_ss = 1
	all_set = sum([copy_wp,copy_ss])

	#start up gazebo if all processes are successful
	if(all_set == 0):
		world = root_dir + 'r{}/'.format(swarmsize) + world_db[world_name]
		loadWorldStr = ('export GAZEBO_MODEL_PATH=$PWD/sources/models;'
					'export GAZEBO_MASTER_URI=http://127.0.0.1:{};'
					'{} --verbose {}'.format(port_number,gzmode,world))
		world_governorStr = ('export GAZEBO_MODEL_PATH=$PWD/sources/models;'
					'export GAZEBO_MASTER_URI=http://127.0.0.1:{};'
					'./compiled_plugins/start_simulation {} {} {} {}'
					.format(port_number, simulation_folder, params_file,
						params_line, simulation_prefix)
					)

		with ThreadPoolExecutor(max_workers=2) as executor:
			gzsimulation = executor.submit(os.system,loadWorldStr)
			gzgovernor = executor.submit(os.system,world_governorStr)
			if gzsimulation.done():
				sys.stderr('Simulation Ended\n')
				gzgovernor.cancel()
				sys.stderr('World Governor thread killed\n')
		
	else:
		print(all_set)

if __name__=='__main__':
	# params_line and port_number vary based on $SGE_TASK_ID value
	print()
	print()
	# print(sys.argv)
	# print(len(sys.argv))
	start_simulation(world_name = sys.argv[1], swarmsize  = sys.argv[2], 
		params_file = sys.argv[3], params_line = sys.argv[4], 
		simulation_folder = sys.argv[5], simulation_prefix = sys.argv[6],
		port_number = sys.argv[7], gzmode = sys.argv[8])