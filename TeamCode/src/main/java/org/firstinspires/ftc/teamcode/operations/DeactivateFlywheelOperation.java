package org.firstinspires.ftc.teamcode.operations;

public class DeactivateFlywheelOperation extends Operation {
    public DeactivateFlywheelOperation(Operation... futureOps) {
        super("Powering flywheel", 1f, futureOps);
    }

    @Override
    public int operate(double dt) {
        this.robot.setFlywheelPower(0);
        return -1;
    }
}
