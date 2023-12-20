--lua
sim = require'sim'
simROS = require'simROS'

function sysCall_init()
    SKIP_FIRST_SHOTCRETE = 20 -- avoid shotcreting at the wrong place
    INIT_POS_STATE = 2 -- init pos_state to avoid timestamp

    new_file = false
    is_shotcreting = false
    skip_shotcreting = SKIP_FIRST_SHOTCRETE
    id_pos_increment = 0

    -- Handle
    jointHandles = {}
    linkHandles = {}
    for i=1,7,1 do
        jointHandles[i] = sim.getObject('./joint', {index = i - 1})
        linkHandles[i] = sim.getObject('./link'..tostring(i+1)..'_resp')
    end

    shotcreteScriptHandle = sim.getScript(
        sim.scripttype_childscript, sim.getObject('/robot_base_footprint_visual/ShotcreteNozzle')
    )

    -- Init robot and ros
    init_robot()
    set_ros_msgs()
end

function getContentFile(filepath)
    if not filepath then
        print("Could not open file")
        return
    end

    pos_state = INIT_POS_STATE
    return parseCSV(filepath)
end

function init_robot()
    local initPose = {0.004, 0.691, -0.002, -1.37, 0.021, -0.522, -0.019}

    for i=1,7,1 do
        sim.setJointPosition(jointHandles[i], initPose[i])
        sim.setJointMode(jointHandles[i], sim.jointmode_dynamic, 0)
        sim.setObjectInt32Param(jointHandles[i], sim.jointintparam_dynctrlmode, sim.jointdynctrl_position)

        sim.setJointTargetPosition(jointHandles[i], initPose[i])
    end
end

function set_ros_msgs()
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos,"ROS interface was found.")
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
end

function sysCall_sensing()
    if new_file then
        csvContent = getContentFile(csv_file)
        new_file = false
    end
end

function sysCall_actuation()
    if not simROS then
        return
    end

    -- Gravity compensation on the robot
    local gravityVect = sim.getArrayParameter(sim.arrayparam_gravity)

    for i=1,7,1 do
        local res, mass = sim.getObjectFloatParameter(linkHandles[i], 3005)
        local force = {-gravityVect[1]*mass, -gravityVect[2]*mass, -gravityVect[3]*mass}
        sim.addForceAndTorque(linkHandles[i],force,{0, 0, 0})
    end


    -- Set targets
    if is_shotcreting then
        if skip_shotcreting == 0 then
            sim.callScriptFunction("setShotcreteState", shotcreteScriptHandle, is_shotcreting)
        else
            skip_shotcreting = skip_shotcreting - 1
        end

        if pos_state < #csvContent then
            for i=1,7,1 do
                sim.setJointTargetPosition(jointHandles[i], csvContent[pos_state][i+1])
            end

            pos_state = pos_state + id_pos_increment
        else
            is_shotcreting = false
            skip_shotcreting = SKIP_FIRST_SHOTCRETE

            pos_state = INIT_POS_STATE
            sim.callScriptFunction("setShotcreteState", shotcreteScriptHandle, is_shotcreting)
        end
    end
end

function sysCall_cleanup()
end

function parseCSV(filePath)
    local file = io.open(filePath, "r")
    if not file then
        error("Could not open file: " .. filePath)
    end

    local result = {}

    for line in file:lines() do
        local row = {}
        for value in line:gmatch("[^,]+") do
            table.insert(row, value)
        end
        table.insert(result, row)
    end

    file:close()
    return result
end

function setCsvFile(filepath)
    if filepath ~= csv_file then
        csv_file = filepath
        new_file = true
    end
end

function setShotcreteState(new_state)
    is_shotcreting = new_state
end

function setIdPosIncrement(new_increment)
    id_pos_increment = new_increment
end

function isShotcreting()
    return is_shotcreting
end
