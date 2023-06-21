set -e

colcon build
source ./install/setup.sh

./utils/MapGen-x86_64.AppImage

mv ./conf ./utils/
mv ./map.sdf ./utils/

conf='./utils/conf'

cp ./utils/map.sdf ./src/simulator/models/mindstorm_map/

ros2 launch simulator sim.launch.py $(<"$conf")
