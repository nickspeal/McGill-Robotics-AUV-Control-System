# This custom message type is to be used to debug the controls
# It contains values published by controls that should be useful
# for debugging the control system


# Estimated Positions

float64 estimated_x
float64 estimated_y
float64 estimated_depth
float64 estimated_yaw
float64 estimated_pitch

# Errors

VectorPID xError
VectorPID yError
VectorPID depthError
VectorPID pitchError
VectorPID yawError


# PID Forces (and Torques)

VectorPID xForce
VectorPID yForce
VectorPID depthForce
VectorPID pitchForce
VectorPID yawForce