package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class TurnOperation extends Operation {

    double angle;
    double power;
    public TurnOperation(String displayName, double angle, double power, float maxRuntime, Operation... futureOps)  {
        super(displayName, maxRuntime, futureOps);
        this.angle = angle;
        this.power = power;
    }

    public void init()  {
        super.init();
        robot.changeTurnAngleDegrees(angle);
    }

    @Override
    public int operate(double dt) {
//        this.robot.turnToTarget(power);
        robot.driveWheels(power,power,-power,-power); // turns
        return -1;
    }
}
