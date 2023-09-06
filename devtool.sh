#!/bin/bash

function rpl(){
    service="gzserver"
    
    if pgrep -x "$service" >/dev/null; then
        echo "$service is running. Exiting..."
        return
    fi

    command="$(echo "$1" | tr '[:upper:]' '[:lower:]')"
    case $command in
        clean)
            rm -rf ./install ./build ./log
            rm -rf ./utils/conf ./utils/map.sdf
            AMENT_PREFIX_PATH='/opt/ros/humble'
            CMAKE_PREFIX_PATH='' 
            ;;
        build)
            
            if [ -z "$2" ]; then
                colcon build
            else
                colcon build --packages-select $2
            fi

            if [ $? -ne 0 ]; then
                return
            fi

            source ./install/setup.sh
            ;;
        mapgen)
            ./utils/MapGen-x86_64.AppImage
            if [[ -f ./conf ]]; then
                mv ./conf ./utils/
                mv ./map.sdf ./utils/
                cp ./utils/map.sdf ./src/simulator/models/mindstorm_map/
            fi
            ;;
        run)
            
            rpl build $2
            
            if [ $? -ne 0 ]; then
                return
            fi
            
            if [[ -f ./utils/conf ]]; then
                conf='./utils/conf'
                ros2 launch simulator sim.launch.py $(<"$conf")
            else 
                ros2 launch shelfino_gazebo multi_shelfino.launch.py
            fi
            ;;
        all)    
            rpl clean
            rpl mapgen
            rpl run
            ;;
        demo)
            if [ -z "$2" ]; then
                return
            fi
            rm ./utils/conf ./utils/map.sdf 
            cp $2/conf $2/map.sdf ./utils
            cp ./utils/map.sdf ./src/simulator/models/mindstorm_map/
            

            rpl run simulator
            if [ $? -ne 0 ]; then
                return
            fi
            ;;
        shortlist)
            echo clean build mapgen run all
            ;;
        *)
            echo "Available commands"
            echo ""
            echo -e "\t rpl clean  : removes install, log, build and files in utils"
            echo -e "\t rpl build  : performs build packages and source environment"
            echo -e "\t rpl demo   : run simulation of prebuilt demos"
            echo -e "\t rpl mapgen : calls MapGen to generate custom map"
            echo -e "\t rpl run    : run simulation"
            echo -e "\t rpl all    : clean -> mapgen -> build -> run"
        ;;
    esac
}

function _rpl() {
    local AVAILABLE_COMMANDS=$(rpl shortlist)
    COMPREPLY=()
    if [ "$COMP_CWORD" -eq 1 ] ; then
        local cur=${COMP_WORDS[COMP_CWORD]}
        COMPREPLY=($( compgen -W "$AVAILABLE_COMMANDS" -- $cur ))
    fi

}

complete -o nospace -F _rpl rpl
