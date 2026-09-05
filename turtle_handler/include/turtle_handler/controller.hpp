#ifndef TURTLE_HANDLER_CONTROLLER_HPP
#define TURTLE_HANDLER_CONTROLLER_HPP

namespace turtle_handler{
    struct Pose
    {
        double x;
        double y;
        double theta;
        double v_lin;
        double v_ang;
    };

    struct MoveInstruction
    {
        double x;
        double y;
        double theta;
        double velocity;
        double zone_data;
    };
    
    struct Command
    {
        double v_lin;
        double v_ang;
    };

    Command computeVelCmd(const Pose& current_pose, const MoveInstruction& instruction);
    
}
#endif
