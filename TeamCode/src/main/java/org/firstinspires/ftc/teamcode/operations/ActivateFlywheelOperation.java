package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ActivateFlywheelOperation extends Operation {
    public ActivateFlywheelOperation(Operation... futureOps) {
        super("Powering flywheel", 1f, futureOps);
    }

    @Override
    public int operate(double dt) {
//        this.robot.setFlywheelPower(RobotHardware.DEFAULT_FLYWHEEL_POWER);
        this.robot.setFlywheelPower(-0.9);
        return -1;
    }
}
