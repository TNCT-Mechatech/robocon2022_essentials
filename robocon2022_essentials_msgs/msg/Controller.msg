Header header

###########################
# Movement
# movement mode: 0-speedy, 1-normal, 2-slow
int32 movement_mode
# movement vel: x, y, yaw(z-direct)
# range -1.0 - 1.0
geometry_msgs/Twist movement_vel

###########################
# Shooter
# shooter num: selected shooter
int32 shooter_num
# shooter power: range 0.0 - 1.0 (float)
float32 shooter_power
# shooter action: 0-nothing, 1-up, 2-down, 3-shot
int32 shooter_action
# ALL shooter reload
bool all_reload

