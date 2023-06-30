
function(register_executable target src dependencies)
    add_executable(${target} ${src})
    ament_target_dependencies(${target} ${dependencies})
endfunction()