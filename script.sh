# display errors
set -e

# call map generator
./utils/MapGen-x86_64.AppImage

if [[ -f ./conf ]] || [[ ! -z "$1" ]]; then
    if [[ -f ./conf ]]; then
        mv ./conf ./utils/
        mv ./map.sdf ./utils/
        cp ./utils/map.sdf ./src/simulator/models/mindstorm_map/
    fi
    conf='./utils/conf'

    colcon build
    source ./install/setup.sh
    ros2 launch simulator sim.launch.py $(<"$conf")
else
    colcon build
    source ./install/setup.sh
    ros2 launch shelfino_gazebo multi_shelfino.launch.py
fi

