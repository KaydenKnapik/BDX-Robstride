import robstride.client
import can
import numpy as np
import time

CAN_PORT = 'can0'
MOTOR_IDS = [16]  # List of motor IDs to control
FULL_ROTATION = 2 * 3.14159

def set_zero_position(client, motor_ids):
    """Set the current position of each motor as zero."""
    for motor_id in motor_ids:
        # Read the current mechanical position
        current_angle = client.read_param(motor_id, 'mechpos')
        # Set the current position as zero by subtracting it
        client.write_param(motor_id, 'loc_ref', current_angle)
        print(f"Motor {motor_id} zeroed at angle: {current_angle:.2f} radians")

with can.interface.Bus(interface='socketcan', channel=CAN_PORT) as bus:
    client = robstride.client.Client(bus)
    
    # Motion parameters
    TARGET_VELOCITY = 8  # Changed from 4 to 8 rotations per second
    ACCEL_TIME = 1.0    # seconds to reach target velocity
    UPDATE_RATE = 100    # Hz
    DELAY_BETWEEN_STEPS = 1/UPDATE_RATE
    
    def calculate_position_profile(start_pos):
        positions = []
        
        # Calculate number of steps for each phase
        accel_steps = int(ACCEL_TIME * UPDATE_RATE)
        
        # Calculate total distance covered during acceleration
        # Using s = (1/2)at^2, where final velocity = at
        accel_distance = 0.5 * TARGET_VELOCITY * ACCEL_TIME
        
        # Acceleration phase
        for i in range(accel_steps):
            t = i * DELAY_BETWEEN_STEPS
            # Distance = (1/2)at^2, where a = TARGET_VELOCITY/ACCEL_TIME
            position = start_pos + 0.5 * (TARGET_VELOCITY/ACCEL_TIME) * (t**2)
            positions.append(position * FULL_ROTATION)
        
        # Constant velocity phase - run for 2 seconds
        constant_time = 0.0  # seconds
        constant_steps = int(constant_time * UPDATE_RATE)
        
        for i in range(constant_steps):
            t = i * DELAY_BETWEEN_STEPS
            position = start_pos + accel_distance + TARGET_VELOCITY * t
            positions.append(position * FULL_ROTATION)
        
        # Deceleration phase
        decel_steps = accel_steps
        final_constant_position = positions[-1] / FULL_ROTATION
        
        for i in range(decel_steps):
            t = i * DELAY_BETWEEN_STEPS
            # Mirror of acceleration, but starting from final constant velocity position
            decel_position = final_constant_position + (TARGET_VELOCITY * t - 
                0.5 * (TARGET_VELOCITY/ACCEL_TIME) * (t**2))
            positions.append(decel_position * FULL_ROTATION)
            
        return positions
    
    # Set control mode to location control for all motors
    print("Setting motors to location control mode...")
    for motor_id in MOTOR_IDS:
        client.write_param(motor_id, 'run_mode', robstride.client.RunMode.Position)

    try:
        # Enable all motors
        print("Enabling motors...")
        for motor_id in MOTOR_IDS:
            client.enable(motor_id)

        # Set zero position
        print("Setting zero position for motors...")
        set_zero_position(client, MOTOR_IDS)
        time.sleep(0.5)  # Short delay to ensure settings take effect
        
        # Get starting positions for each motor
        start_positions = {motor_id: client.read_param(motor_id, 'loc_ref') / FULL_ROTATION 
                          for motor_id in MOTOR_IDS}
        
        # Calculate position profiles for each motor
        motor_positions = {motor_id: calculate_position_profile(start_pos) 
                          for motor_id, start_pos in start_positions.items()}
        
        # Execute motion profile
        print(f"Starting motion profile (target velocity: {TARGET_VELOCITY} rot/s)...")
        for i in range(len(motor_positions[MOTOR_IDS[0]])):  # Use first motor's profile length
            # Write positions to all motors
            for motor_id in MOTOR_IDS:
                client.write_param(motor_id, 'loc_ref', motor_positions[motor_id][i])
                angle = client.read_param(motor_id, 'mechpos')
                # Wrap the angle to 0-2π range
                wrapped_angle = angle % FULL_ROTATION
                rotations = angle / FULL_ROTATION
                wrapped_rotations = wrapped_angle / FULL_ROTATION
                print(f"Motor {motor_id} position: {wrapped_rotations:.3f} rotations ({wrapped_angle:.2f} radians) [Total: {rotations:.3f} rotations]")

            time.sleep(DELAY_BETWEEN_STEPS)
    
    finally:
        # Ensure all motors are disabled even if there's an error
        print("Disabling motors...")
        for motor_id in MOTOR_IDS:
            client.disable(motor_id)
