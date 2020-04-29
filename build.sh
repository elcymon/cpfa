plugins_path=$PWD/compiled_plugins

cd sources/plugins/build
cmake ../
make

cp -f */*.so start_simulation/start_simulation $plugins_path