package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Operations to turn the robot to a specific target angle.
 */
public class RelativeTurnOperation extends Operation {

    double angle;
    double power;
    public RelativeTurnOperation(String displayName, double deltaAngle, double power, float maxRuntime, Operation... futureOps)  {
        super(displayName, maxRuntime, futureOps);
        this.angle = deltaAngle;
        this.power = power;
    }

    public void init()  {
        super.init();
        robot.clearTargetAngle();
        robot.changeTurnAngleDegrees(angle);
    }

    @Override
    public int operate(double dt) {
        robot.turnToTarget(power);
        return robot.isAtTargetAngle() ? 0 : -1;
    }
}
