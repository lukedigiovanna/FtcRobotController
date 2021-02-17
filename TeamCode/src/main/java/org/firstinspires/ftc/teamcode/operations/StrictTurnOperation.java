package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Operations to turn the robot to a specific target angle.
 */
public class StrictTurnOperation extends Operation {

    double angle;
    double power;
    public StrictTurnOperation(String displayName, double targetAngle, double power, float maxRuntime, Operation... futureOps)  {
        super(displayName, maxRuntime, futureOps);
        this.angle = -targetAngle;
        this.power = power;
    }

    public void init()  {
        super.init();
        robot.setTargetAngleDegrees(angle);
        robot.setTargetAngleThreshold(Math.PI/64);
    }

    @Override
    public int operate(double dt) {
        robot.turnToTarget(power);
        if (robot.isAtTargetAngle()) {
            robot.resetTargetAngleThreshold();
            return 0;
        } else {
            return -1;
        }
    }
}
