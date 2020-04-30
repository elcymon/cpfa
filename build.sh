plugins_path=$PWD/compiled_plugins
root_dir=$PWD
cd $root_dir/sources/plugins/start_simulation/build
cmake ../
make
cp -f start_simulation $plugins_path

cd $root_dir/sources/plugins/wp_swarm/build
cmake ../
make
cp -f libwp_swarm.so $plugins_path