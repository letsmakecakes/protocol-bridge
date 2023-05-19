import time
import pygame
from pymavlink import mavutil

# Set up joysticks
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

master = mavutil.mavlink_connection('192.168.0.105:14550')
master.wait_heartbeat()

yaw_flag = 0
hover_flag = 0
# Loop to read joystick input and send control commands
while True:
    pygame.event.get()
    roll = int(joystick.get_axis(0) * 1000)
    pitch = int(joystick.get_axis(1) * 1000)
    yaw = int(joystick.get_axis(2) * 1000)
    throttle = int(joystick.get_axis(3) * 1000)
    # roll = 0 if roll <=300 and roll >=-300 else roll
    # pitch = 0 if pitch <=300 and pitch >=-300 else pitch
    # yaw = 0 if yaw <=300 and yaw >=-300 else yaw
    # throttle = 0 if throttle <=300 and throttle >=-300 else throttle
    loiter_mode = joystick.get_button(0)
    arm = joystick.get_button(1)
    disarm = joystick.get_button(2)
    flag_button = joystick.get_button(4)
    hover_button = joystick.get_button(5)

    if(flag_button):
        yaw_flag = 0 if yaw_flag == 1 else 1
        if yaw_flag == 0:
            print("yaw axis is on")
        else:
            print("yaw axis is off")

    if(hover_button):
        hover_flag = 0 if hover_flag == 1 else 1
        if hover_flag == 0:
            print("hover is off")
        else:
            print("hover is on")

    if(hover_flag):
        throttle = -500

    if(loiter_mode):
        mode = 'LOITER'
        print(f"setting the drone in {mode} mode")
        print(master.mode_mapping())
        if mode not in master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(master.mode_mapping().keys()))
            sys.exit(1)
        mode_id = master.mode_mapping()[mode]
        master.set_mode(mode_id) 
        while True:
            ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()
            if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                continue
            print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            break


    if(arm):
        print("Arming the Drone")

        # master.mav.command_long_send(
        #     master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,21196, 0, 0, 0, 0, 0)

        master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                     0, 1, 21196, 0, 0, 0, 0, 0) 
        while True:
            msg = master.recv_msg()
            if msg and msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if  msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Drone is armed")
                    break
                else:
                    print("Failed to arm the Drone")
                    # exit()
                    break

    if(disarm):
        print("Disarming the Drone")

        master.mav.command_long_send(
            master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,21196, 0, 0, 0, 0, 0)


        while True:
            msg = master.recv_msg()
            if msg and msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if  msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Drone is disarmed")
                    break
                else:
                    print("Failed to disarm the Drone")
                    # exit()
                    break


    # Send control commands
    if(yaw_flag):
        master.mav.manual_control_send(
            master.target_system,
            roll, pitch,-throttle,0,
            0)  # set to 0 to indicate that buttons are not being used
        
    else:
        master.mav.manual_control_send(
            master.target_system,
            roll, pitch, -throttle, yaw,
            0)  # set to 0 to indicate that buttons are not being used

    # print(roll,pitch,throttle,yaw)
    # master.mav.manual_control_send(
    #     master.target_system,
    #     roll, pitch, yaw, throttle,
    #     0)  # set to 0 to indicate that buttons are not being used master.mav.manual_control_send(
    # print(roll,pitch ,yaw,throttle)
    time.sleep(1)  # sleep for a short time to avoid overloading the serial port
