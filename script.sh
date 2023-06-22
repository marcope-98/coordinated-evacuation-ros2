# display errors
set -e

# map generation sub routine
./utils/MapGen-x86_64.AppImage

# mv files in the code base
if test -f ./conf; then
    mv ./conf ./utils/
fi

if test -f ./map.sdf; then
    mv ./map.sdf ./utils/
    cp ./utils/map.sdf ./src/simulator/models/mindstorm_map/
fi

conf='./utils/conf'

colcon build
source ./install/setup.sh

if test -f "${conf}"; then
    ros2 launch simulator sim.launch.py $(<"$conf")
else
    echo "default"
fi