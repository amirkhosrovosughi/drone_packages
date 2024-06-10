# drone_msg/msg/DroneDirectionCommand.msg

uint8 NO_COMMAND         = 0 # NO COMMAND
uint8 GO_TO_ORIGIN       = 1 # Hover on origin
uint8 GO_TO              = 2 # move to give position
uint8 HEAD_TO            = 3 # Head towards the given value
uint8 STOP_HERE          = 4 # Hover at correct position
uint8 HEAD_FORWARD       = 5 # Head to zero angle in current setpoint
uint8 HEAD_BACKWARD      = 6 # Head to pi angle in current setpoint
uint8 HEAD_LEFT          = 7 # Head to - pi /2 angle in current setpoint
uint8 HEAD_RIGHT         = 8 # Head to pi / 2 angle in current setpoint


uint8 cli_command         # CLI move command
string cli_command_value  # CLI move command value

float32 vertical_step  # Range: [-10, 10], represents size of max vertical step
float32 forward_step   # Range: [-10, 10], represents size of max forward step
float32 lateral_step   # Range: [-10, 10], represents size of max lateral step
float32 angular_step   # Range: [-10, 10], represents size of max angular step

