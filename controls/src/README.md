These two files contain the bulk of the logic of the whole package. Peripheral files can be found in the directories above.

The controls node implements PID controllers on 5 DOF of the vehicle, computing the net force and torque that should be applied in order to close the gap between the desired setpoint and the estimated state. Closed loop position control and open loop speed control are implemented.

The thrust_mapper node converts the wrench message, containing the net force and torque to be exerted, into motor commands for each of the six thrusters. It accounts for the geometry of the vehicle and the experimentally determined thrust/voltage and voltage/command mappings of the thrusters and motor controllers, respectively.
