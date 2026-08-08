std_msgs/Header header
string joint_name
float64 position
float64 velocity
float64 torque
uint8 status_code
# Motor temperatures in degrees C. NaN when the motor has not reported yet.
# temperature_mos is the driver/MOSFET sensor, temperature_rotor the rotor winding.
float64 temperature_mos
float64 temperature_rotor
