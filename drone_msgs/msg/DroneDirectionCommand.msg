# drone_msg/msg/DroneDirectionCommand.msg
bool stop            # Howering at current position
float32 vertical_step   # Range: [-10, 10], represents size of max vertical step
float32 forward_step   # Range: [-10, 10], represents size of max forward step
float32 lateral_step   # Range: [-10, 10], represents size of max lateral step
float32 angular_step   # Range: [-10, 10], represents size of max angular step

