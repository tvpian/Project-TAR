#!/bin/bash

# Function to calculate sinusoidal movement
sinusoidal_move() {
    amplitude=30   # Increase the amplitude for longer distances
    frequency=0.05 # Reduce the frequency for slower movement
    time=$(date +%s%N)  # Get current time in nanoseconds
    sine_value=$(echo "s($frequency * $time / 1000000000)" | bc -l)
    movement=$(echo "$amplitude * $sine_value" | bc -l)
    echo $movement
}

# Trap Ctrl+C to exit the script gracefully
trap 'echo "Exiting script"; exit' INT

while true; do
    x_movement=$(sinusoidal_move)

    # Set the position of the model using gz service
    gz service -s /world/baylands/set_pose \
        --reqtype gz_msgs.Pose \
        --reptype gz_msgs.Boolean \
        --timeout 300 \
        --req "name: 'joe_apriltag', position: {x: $x_movement, y: 0, z: 1}"

    # Sleep for 0.5 second (2 times per second)
    sleep 0.25
done

