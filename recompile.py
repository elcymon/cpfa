import subprocess
import sys

def compile_plugins():
	copy_wp=None
	copy_rp=None
	copy_np=None
	copy_wg=None
	print('compiling world plugin')
	world_plugin = subprocess.run("cd sources/w_swarm1/build;rm -rf *;cmake ../;make",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	if world_plugin.returncode:
		print('************world plugin compilation failed')
		print(world_plugin.stderr)
	else:
		print('world plugin compilation successful')
		copy_wp = subprocess.run("cp sources/w_swarm1/build/libwp_swarm1.so ./compiled_plugins",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	print()

	print('compiling robot plugin')
	robot_plugin = subprocess.run("cd sources/mp_cpfa/build;rm -rf *;cmake ../;make",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	if robot_plugin.returncode:
		print('************robot plugin compilation failed')
		print(robot_plugin.stderr)
	else:
		print('robot plugin compilation successful')
		copy_rp = subprocess.run("cp sources/mp_cpfa/build/libmp_cpfa.so ./compiled_plugins",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	print()

	print('compiling nest plugin')
	nest_plugin = subprocess.run("cd sources/m_nest/build;rm -rf *;cmake ../;make",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	if nest_plugin.returncode:
		print('************nest plugin compilation failed')
		print(nest_plugin.stderr)
	else:
		print('nest plugin compilation successful')
		copy_np = subprocess.run("cp sources/m_nest/build/libnest_plugin.so ./compiled_plugins",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	print()

	print('compiling world governor')
	world_governor = subprocess.run("cd sources/world_governor/build;rm -rf *;cmake ../;make",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
	if world_governor.returncode:
		print('*************world governor compilation failed')
		print(world_governor.stdout)
	else:
		print('world governor compilation successful')
		subprocess.run("rm world_governor",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
		copy_wg = subprocess.run("cp sources/world_governor/build/world_governor .",stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.PIPE,shell=True)
		print(copy_wg.stderr)
		

if __name__=='__main__':
	compile_plugins()