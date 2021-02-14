package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Operations to turn the robot to a specific target angle.
 */
public class TurnOperation extends Operation {

    double angle;
    double power;
    public TurnOperation(String displayName, RobotHardware robot, double targetAngle, double power, float maxRuntime)  {
        super(displayName, robot, maxRuntime);
        this.angle = -targetAngle;
        this.power = power;
    }

    public void init()  {
        super.init();
        robot.setTargetAngleDegrees(angle);
    }

    @Override
    public int operate(double dt) {
        this.robot.turnToTarget(power);
        return this.robot.isAtTargetAngle() ? 0 : -1;
    }
}
