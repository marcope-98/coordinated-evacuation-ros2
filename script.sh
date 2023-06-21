# display line and print errors
set -xe

# variables
conf='./utils/conf'

# map generation sub routine
./utils/MapGen-x86_64.AppImage

# mv files in the code base
mv ./conf ./utils/
mv ./map.sdf ./utils/
cp ./utils/map.sdf ./src/simulator/models/mindstorm_map/

# build source and run
colcon build
source ./install/setup.sh
ros2 launch simulator sim.launch.py $(<"$conf")
