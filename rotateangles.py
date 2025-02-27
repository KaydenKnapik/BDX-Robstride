import robstride.client
import can
import time

CAN_PORT = 'can0'
MOTOR_IDS = [127, 17]  # Control all motors
FULL_ROTATION = 2 * 3.14159

# Define target positions for motors 16
POSITIONS = [23, 153]  

# Define target positions for motor 127
POSITIONS_127 = [236, 359]  
POSITIONS_17 = [144, 240]

WAIT_TIME = 0.1  # Wait time at each position

def set_zero_position(client, motor_ids):
    """Set the current position of each motor as zero."""
    for motor_id in motor_ids:
        current_angle = client.read_param(motor_id, 'mechpos')
        client.write_param(motor_id, 'loc_ref', current_angle)
        client.write_param(motor_id, 'limit_spd', 50.0)
        print(f"Motor {motor_id} zeroed at angle: {current_angle:.2f} radians")
        time.sleep(0.1)

def move_to_positions(client, motor_positions):
    """Move multiple motors to specific positions in degrees."""
    targets = {}
    
    # Convert target degrees to radians and send commands
    for motor_id, target_deg in motor_positions.items():
        target_rad = (target_deg * FULL_ROTATION) / 360.0
        client.write_param(motor_id, 'loc_ref', target_rad)
        targets[motor_id] = target_deg  # Store target degrees for monitoring

    # Wait for all motors to reach position
    start_time = time.time()
    while True:
        all_reached = True
        for motor_id, target_deg in targets.items():
            current_angle = client.read_param(motor_id, 'mechpos')
            degrees = (current_angle * 360.0) / FULL_ROTATION
            print(f"Motor {motor_id} position: {degrees:.1f}°", end='\r', flush=True)

            if abs(degrees - target_deg) > 2:  # Check if still moving
                all_reached = False

        if all_reached or (time.time() - start_time > 1.5):  # Timeout
            break
        
        time.sleep(0.05)



with can.interface.Bus(interface='socketcan', channel=CAN_PORT) as bus:
    client = robstride.client.Client(bus)
    
    print("Setting motors to position control mode...")
    for motor_id in MOTOR_IDS:
        client.write_param(motor_id, 'run_mode', robstride.client.RunMode.Position)

    try:
        print("Enabling motors...")
        for motor_id in MOTOR_IDS:
            client.enable(motor_id)
            client.write_param(motor_id, 'limit_spd', 50.0)
            client.write_param(motor_id, 'loc_kp', 20.0)
            client.write_param(motor_id, 'spd_kp', 5.0)

        print("Setting zero position for motors...")
        set_zero_position(client, MOTOR_IDS)
        time.sleep(0.5)

        print("\nStarting movement for motors...")
        while True:  # Loop until Ctrl+C
            for i in range(len(POSITIONS)):  
                pos_17 = POSITIONS_17[i]
                pos_127 = POSITIONS_127[i]

                print(f"\nMoving motors 17 to {pos_17}° and motor 127 to {pos_127}°...")
        
                # Move motors simultaneously
                move_to_positions(client, {127: pos_127, 17: pos_17})

                print(f"\nReached positions, waiting for {WAIT_TIME} seconds...")
                time.sleep(WAIT_TIME)


    except KeyboardInterrupt:
        print("\nStopping motor movement...")

    finally:
        print("Disabling motors...")
        for motor_id in MOTOR_IDS:
            client.disable(motor_id)
