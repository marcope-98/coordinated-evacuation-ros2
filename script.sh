# display errors
set -e


command="$(echo "$1" | tr '[:upper:]' '[:lower:]')"
case $command in
    clean)
        rm -rf ./install ./build ./log
        rm -rf ./utils/conf ./utils/map.sdf
        ;;
    build)
        colcon build
        source ./install/setup.sh
        ;;
    mapgen)
        ./utils/MapGen-x86_64.AppImage
        if [[ -f ./conf ]]; then
            mv ./conf ./utils/
            mv ./map.sdf ./utils/
            cp ./utils/map.sdf ./src/simulator/models/mindstorm_map/
            conf='./utils/conf'
        fi
        ;;
    repeat)
        colcon build
        source ./install/setup.sh
        if [[ -f ./utils/conf ]]; then
            ros2 launch simulator sim.launch.py $(<"$conf")
        else 
            ros2 launch shelfino_gazebo multi_shelfino.launch.py
        fi
        ;;
    all)
        rm -rf ./install ./build ./log
        rm -rf ./utils/conf ./utils/map.sdf
        
        ./utils/MapGen-x86_64.AppImage
        if [[ -f ./conf ]]; then
            mv ./conf ./utils/
            mv ./map.sdf ./utils/
            cp ./utils/map.sdf ./src/simulator/models/mindstorm_map/
            conf='./utils/conf'
            colcon build
            source ./install/setup.sh
            ros2 launch simulator sim.launch.py $(<"$conf")
        else
            colcon build
            source ./install/setup.sh
            ros2 launch shelfino_gazebo multi_shelfino.launch.py
        fi
        ;;
    *)
        echo -e "\n"
        echo "Use script.sh as follows:"
        echo ""
        echo -e "\t ./script.sh clean  : removes install, log, build and files in utils/"
        echo -e "\t ./script.sh build  : performs build packages and source environment"
        echo -e "\t ./script.sh mapgen : calls MapGen to generate custom map"
        echo -e "\t ./script.sh repeat : repeats last simulation"
        echo -e "\t ./script.sh all    : remove folders and MapGen files, calls MapGen, if MapGen is exited without exporting a map it defaults to multi_shelfino.launch.py"
        echo
        echo -e "\t The arguments are case insensitive, i.e. ./script.sh ClEaN is valid"
        echo -e "\n"
    ;;
esac
