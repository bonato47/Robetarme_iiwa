--lua
sim = require'sim'
simROS = require'simROS'

nb_dofs = NB_DOFS

function sysCall_init()
    -- Robot state
    jointNames = {}
    jointPosition = {}
    jointVelocity = {}
    jointTorque = {}

    -- Handle
    get_handles()

    -- Init robot and ros
    set_ros_msgs()
end

function set_ros_msgs()
    -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)
    pub_joint_pose = simROS.advertise('/sim/ee_info/Pose', 'geometry_msgs/Pose')
    simROS.publisherTreatUInt8ArrayAsString(pub_joint_pose)

    pub_joint_vel = simROS.advertise('/sim/ee_info/Vel', 'geometry_msgs/Twist')
    simROS.publisherTreatUInt8ArrayAsString(pub_joint_vel)

    pub_clock = simROS.advertise('/clock', 'std_msgs/Time')

    pub_joint_states = simROS.advertise('/sim/joint_states', 'sensor_msgs/JointState')
    simROS.publisherTreatUInt8ArrayAsString(pub_joint_states)

    sub_joint_cmd = simROS.subscribe('/sim/CONTROLLER_TYPE_controller/command', 'std_msgs/Float64MultiArray', 'callback_joint_cmd')
    simROS.subscriberTreatUInt8ArrayAsString(sub_joint_cmd)
end

function sysCall_sensing()
    -- Get end effector pose
    ee_pose = {}
    sim_ee_pose = sim.getObjectPose(eeHandle, baseHandle)

    ee_pose.position = {x=sim_ee_pose[1], y=sim_ee_pose[2], z=sim_ee_pose[3]}
    ee_pose.orientation = {
        x=sim_ee_pose[4], y=sim_ee_pose[5], z=sim_ee_pose[6], w=sim_ee_pose[7]
    }

    -- Get end effector velocity
    ee_twist = {}
    sim_ee_linvel, sim_ee_angvel = sim.getObjectVelocity(eeHandle)

    ee_twist.linear = {x=sim_ee_linvel[1], y=sim_ee_linvel[2], z=sim_ee_linvel[3]}
    ee_twist.angular = {x=sim_ee_angvel[1], y=sim_ee_angvel[2], z=sim_ee_angvel[3]}

    simROS.publish(pub_joint_pose, ee_pose)
    simROS.publish(pub_joint_vel, ee_twist)

    -- Get joint states
    for i=1, nb_dofs, 1 do
        jointPosition[i] = sim.getJointPosition(jointHandles[i])
        jointVelocity[i] = sim.getJointVelocity(jointHandles[i])
        jointTorque[i] = sim.getJointForce(jointHandles[i])
    end

    publishJointState()
end

function sysCall_actuation()
    -- Publish simulated time
    simROS.publish(pub_clock, {data = sim.getSimulationTime()})

    -- Gravity compensation on the robot
    local gravityVect = sim.getArrayParameter(sim.arrayparam_gravity)

    for i=1, nb_dofs, 1 do
        local res, mass = sim.getObjectFloatParameter(linkHandles[i], 3005)
        local force = {-gravityVect[1]*mass, -gravityVect[2]*mass, -gravityVect[3]*mass}
        sim.addForceAndTorque(linkHandles[i],force,{0, 0, 0})
    end

    -- Set command target
    if new_cmd then
        set_cmd()
    end
end

function sysCall_cleanup()
end

function publishJointState()
    local joint_state = {}
    joint_state.name = jointNames
    joint_state.position = jointPosition
    joint_state.velocity = jointVelocity
    joint_state.effort = jointTorque

    simROS.publish(pub_joint_states, joint_state)
end

function callback_joint_cmd(msg)
    new_cmd = true
    cmd_msg = msg.data
end

function get_handles()
    JOINT_NAMES
    JOINT_HANDLES
    LINK_HANDLES

    baseHandle = sim.getObject(BASE_HANDLE_NAME)
    eeHandle = sim.getObject(EE_HANDLE_NAME)
end

function set_cmd()
    new_cmd = false

    for i=1, nb_dofs, 1 do
        sim.setJointTargetCMD(jointHandles[i], cmd_msg[i])
    end
end
