package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class TurnOperation extends Operation {

    double angle;
    double power;
    public TurnOperation(String displayName, RobotHardware robot, float maxRuntime, double angle, double power)  {
        super(displayName, robot, maxRuntime);
        this.angle = angle;
        this.power = power;
    }

    public void init()  {
        super.init();
        robot.changeTurnAngle(angle);
    }

    @Override
    public int operate() {
        return -1;
    }
}
