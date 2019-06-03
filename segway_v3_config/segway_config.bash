# This configures the environment variables for a segway rmp V3 robot and simulation
# This is necessary to run before starting the simulation or the hardware interfaces
# it is sourced automatically by the sourcing the workspace or by running the
# upstart service
#

# If there is an onboard PC powered by the system this will run the watchdog
# to make sure it gets gracefully shutdown before the system power cuts out.
export SEGWAY_POWERS_PC_ONBOARD=true

# Joystick input types are:
# - ds4  - The Sony DualShock4 Joystick for PS4 (model: CUH-ZCT2U)
# - f710 - The Logitech F710 Wireless Joystick (in XINPUT mode)
# - f310 - The Logitech F310 Wired Joystick (in XINPUT mode)
# - (not supported yet) x3dp - The Logitech Extreme 3D Pro Joystick (not supported yet)   
export SEGWAY_JOY_TYPE=f710
export SEGWAY_HAS_ONBOARD_JOY=true

# Joystick deadzone (smaller value increases sensitivity to small movements)
export SEGWAY_JOY_DEADZONE=0.1

# Joystick configurations for joystick set SEGWAY_JOY_IS_ATTACHED if the joystick
# is physically attached to this PC
if [ "$HOSTNAME" = sibot1 ]; then
    export SEGWAY_JOY_IS_ATTACHED=true
    export SEGWAY_JOY_DEV=/dev/input/js0
    
else
    export SEGWAY_JOY_IS_ATTACHED=false
    if [ "$SEGWAY_JOY_TYPE" = ds4 ]; then
        export SEGWAY_JOY_DEV=/dev/input/js1
    else
        export SEGWAY_JOY_DEV=/dev/input/js0
    fi
fi



