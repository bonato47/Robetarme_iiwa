
function set_cmd()
    new_cmd = false

    for i=1, nb_dofs, 1 do
        sim.setJointTargetCMD(jointHandles[i], cmd_msg[i])
    end
end
