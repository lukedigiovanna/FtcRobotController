package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.*;

@Deprecated
public class MainRobotDep extends Robot {
    public DcMotor backLeft, backRight, frontLeft, frontRight, flywheel, largeRollers, smallRollers;

    public MainRobotDep(OpMode opMode) {
        super();
        this.setHardware(RobotHardwareDep.getMainRobotHardware(opMode.hardwareMap));
        this.backLeft = this.hardware.getMotor("back_left_chassis");
        this.backRight = this.hardware.getMotor("back_right_chassis");
        this.frontLeft = this.hardware.getMotor("front_left_chassis");
        this.frontRight = this.hardware.getMotor("front_right_chassis");
        DcMotor[] motors = {backLeft, frontLeft, frontRight, backRight};
        for (int i = 0; i < motors.length; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.largeRollers = this.hardware.getMotor("large_rollers");
//        this.flywheel = this.hardware.getMotor("flywheel");
//        this.smallRollers = this.hardware.getMotor("small_rollers");
    }

    public void driveForward(double power) {
        driveWheels(power, power, power, power);
    }

    public boolean driveIsBusy() {
        return this.backLeft.isBusy() && this.backRight.isBusy() && this.frontLeft.isBusy() && this.frontRight.isBusy();
    }

    public void stopChassis() {
        driveWheels(0,0,0,0);
    }

    public void clearEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void vectorStrafe(Vector movementVector, double power)  {
        movementVector.normalize();
        this.relativeStrafe(movementVector.y, movementVector.x, 0, power);
    }

    public void vectorStrafe(Vector movementVector, double power, double turn)  {
        movementVector.normalize();
        this.relativeStrafe(movementVector.y, movementVector.x, turn, power);
    }

    public void relativeStrafe(double robotAngle, double desiredAngle, double magnitude, double turn, double power) {
        double alpha = robotAngle - desiredAngle;
        double drive = magnitude * Math.sin(alpha);
        double strafe = magnitude * Math.cos(alpha);

        this.strafe(drive, strafe, turn, power);
    }

    public void relativeStrafe(double drive, double strafe, double turn, double power) {
        drive *= -1; //invert to handle the desired direction of the robot.
        // get desired angle as drive (y) and strafe (x)
        double angle = Math.atan(drive/strafe);
        if (strafe < 0)
            angle += Math.PI;
        if (strafe == 0) {
            if (drive > 0)
                angle = Math.PI/2;
            else
                angle = Math.PI * 3 / 2;
        }

        // limited between [0, 360)
        double hardAngle = hardware.getAngle();

        // convert degrees to radians
        hardAngle = (hardAngle / 180) * Math.PI;

        // correct for desired angle
        double alpha = hardAngle - angle;

        double mag = Math.sqrt(drive * drive + strafe * strafe);

        // apply trigonometry
        drive = mag * Math.sin(alpha);
        strafe = mag * Math.cos(alpha);

        this.strafe(drive, strafe, turn, power);
    }

    public void strafe(double drive, double strafe, double turn, double power) {
        double lf = drive + turn + strafe;
        double rf = drive - turn + strafe;
        double rb = drive - turn - strafe;
        double lb = drive + turn - strafe;

        //adjust motor powers by a scalar
        lf *= power;
        rf *= power;
        rb *= power;
        lb *= power;

        // drive wheels
        driveWheels(lb,rb,lf,rf);
    }

    public void driveWheels(double lb, double rb, double lf, double rf) {
        this.backLeft.setPower(lb);
        this.backRight.setPower(rb);
        this.frontLeft.setPower(lf);
        this.frontRight.setPower(rf);
    }

    public void intake(boolean on)    {
        if(on)  {
            this.largeRollers.setPower(-1);
//            this.smallRollers.setPower(-1);
        }
        else    {
            this.largeRollers.setPower(0);
//            this.smallRollers.setPower(0);
        }

    }

    public void fire(boolean on)  {
//        if(on)
//            this.flywheel.setPower(-0.42);
//        else
//            this.flywheel.setPower(0);
    }

    public void lift(double power)  {
        //this.lift.setPower(power);
    }

}
