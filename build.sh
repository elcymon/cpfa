plugins_path=$PWD/compiled_plugins

cd sources/plugins/wp_swarm/build
# cmake ../
make

cp -f *.so start_simulation $plugins_path