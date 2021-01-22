package org.firstinspires.ftc.teamcode.hardware;

@Deprecated
public abstract class Robot {
    protected RobotHardwareDep hardware;

    public Robot() {

    }

    public void setHardware(RobotHardwareDep hardware) {
        this.hardware = hardware;
    }

    public RobotHardwareDep getHardware() {
        return this.hardware;
    }
}
