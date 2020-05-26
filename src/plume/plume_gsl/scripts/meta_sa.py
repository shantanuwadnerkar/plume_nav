import rospy


concentration_curr = 0
concentration_prev = 0
simulation_time = 0

# Start with some initial guess. How?

# Define some epsilon based on the sensor configuration
# Random value. Change later
concentration_epsilon = 1e-4

# Define some lamba. Based on what?
# Random value. Change later
probability_threshold = 1e-4

# Subscribe to the sensor readings topic

# Get sensor reading

# Substract the current sensor reading from the previous one
concentration_change = concentration_curr - concentration_prev

if concentration_change >= concentration_epsilon:
    pass
else:
    maintain_direction_prob = exp(concentration_change - concentration_epsilon)/simulation_time


# If the concentration is higher than or equal to epsilon, continue

# else, find the probability that the current direction is right



# If maintain_direction_prob is greater than lambda, continue

# else, get a new direction.



# If the current cell is the source, stop

# else keep going in the current direction

# Update simulation_time










# Store the current sensor reading as the previous sensor reading