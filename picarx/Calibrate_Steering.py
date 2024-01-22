import picarx_improved
import time

def calibrate_steering():
    # Create an instance of the PiCar-X class
    px = picarx_improved.Picarx()

    # Drive forward at zero steering angle
    px.set_dir_servo_angle(3)  # Assuming this sets the steering angle
    px.forward(30)  # Set a moderate speed
    time.sleep(5)  # Run for a short duration
    px.stop()

    # At this point, observe the drift direction

    # Modify the steering calibration based on observation
    # This is a manual step - you need to decide the value based on the observed drift
    # Example: If the car drifts right, decrease the calibration angle
    #new_calibration_value = -5  # Example value, adjust based on observation
    #px.dir_servo_calibrate(new_calibration_value)

    #print("Steering calibration updated.")

if __name__ == "__main__":
    calibrate_steering()