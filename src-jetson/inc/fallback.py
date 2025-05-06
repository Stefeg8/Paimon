from pymavlink import mavutil
import time

def emergency_auto_level(master,hover_thrust=0.5,start_time=time.time()):
    master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask=0b10000000,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=hover_thrust # should be hover thrust
    )

def emergency_land(master,takeoff_thrust=0.5,start_time=time.time()):
        #emergency_auto_level(master,takeoff_thrust,start_time)  # we can consider passing autolevel first
        print("ramping down thrust")
        ramp_duration = 4
        steps = int(ramp_duration / 0.05) 
        thrust_values_up = [i * (takeoff_thrust / steps) for i in range(steps + 1)]
        thrust_values_down = list(reversed(thrust_values_up))
        for thrust in thrust_values_down:
                master.mav.set_attitude_target_send(
                int((time.time() - start_time) * 1000),
                master.target_system,
                master.target_component,
                type_mask=0b10000000,
                q=[1, 0, 0, 0],
                body_roll_rate=0,
                body_pitch_rate=0,
                body_yaw_rate=0,
                thrust=thrust
                )
                time.sleep(0.05)

def emergency_manual_control(master,takeoff_thrust=0.5,start_time=time.time()):
       emergency_auto_level(master,takeoff_thrust,start_time)
       print("switching to manual mode")
       master.mav.command_long_send(
             master.target_system,
             master.target_component,
             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
             0,
             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
             1,  # manual mode number in PX4
             0, 0, 0, 0, 0
             )
       time.sleep(1)
       print("manual mode set")

def emergency_stabilized_control(master,takeoff_thrust=0.5,start_time=time.time()):
       emergency_auto_level(master,takeoff_thrust,start_time)
       print("switching to stablized mode")
       master.mav.command_long_send(
             master.target_system,
             master.target_component,
             mavutil.mavlink.MAV_CMD_DO_SET_MODE,
             0,
             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
             10,  # manual mode number in PX4
             0, 0, 0, 0, 0
             )
       time.sleep(1)
       print("stabilized mode set")

def auto_takeoff(master,start_time=time.time()):
      ramp_duration = 10
      takeoff_thrust = 0.54
      steps = int(ramp_duration / 0.05) 
      thrust_values_up = [i * (takeoff_thrust / steps) for i in range(steps + 1)]
      thrust_values_down = list(reversed(thrust_values_up))

      # Ramp up
      print("ramping up")
      for thrust in thrust_values_up:
            master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask=0b10000000,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=thrust
            )
            time.sleep(0.05)

def auto_land(master, start_time):
       # Ramp down
      ramp_duration = 10
      takeoff_thrust = 0.54
      steps = int(ramp_duration / 0.05) 
      thrust_values_up = [i * (takeoff_thrust / steps) for i in range(steps + 1)]
      thrust_values_down = list(reversed(thrust_values_up))

      print("ramping down thrust")
      for thrust in thrust_values_down:
            master.mav.set_attitude_target_send(
            int((time.time() - start_time) * 1000),
            master.target_system,
            master.target_component,
            type_mask=0b10000000,
            q=[1, 0, 0, 0],
            body_roll_rate=0,
            body_pitch_rate=0,
            body_yaw_rate=0,
            thrust=0.49
            )
            time.sleep(0.05)

      print("hopefully touchdown")
def system_shutdown(master):
      
      # Disarm the drone
      print("disarming")
      master.mav.command_long_send(
      master.target_system,
      master.target_component,
      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      0,
      0, 0, 0, 0, 0, 0
      )
      print("disarmed")

      print("switching back to manual mode")
      master.mav.command_long_send(
      master.target_system,
      master.target_component,
      mavutil.mavlink.MAV_CMD_DO_SET_MODE,
      0,
      mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
      1,
      0, 0, 0, 0, 0
      )
      print("Manual mode activated, shutdown complete")