package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.utils.Vector;
// Epic Code ✅✅✅
public class RobotHardware {

    public DcMotor backLeftDrive, backRightDrive, frontRightDrive, frontLeftDrive;
    private DcMotor flywheel;
    private DcMotor largeRollers, smallRollers;
    private DcMotor slideDrive;
    private Servo loadingServo, wobbleServo;

    private RevColorSensorV3 colorDistance;

    // IMU
    private BNO055IMU imu;
    private Orientation lastAngles;
    private double globalAngle;

    private double targetAngle = 0;

    private double loadingTime = 0.5;
    private double retractTime = 0;
    private double loadedPosition = 0.0;
    private double storingPosition = 0.5;

    public static double
            TICKS_PER_REV = 383.6,
            GEAR_REDUCTION = 2.0,
            WHEEL_DIAMETER_INCHES = 3.858,
            TICKS_PER_INCH = (TICKS_PER_REV * GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);

    public static final double DEFAULT_FLYWHEEL_POWER = -0.75;

    public RobotHardware(HardwareMap hardware) {
        frontLeftDrive  = hardware.get(DcMotor.class, "front_left_chassis");
        frontRightDrive = hardware.get(DcMotor.class, "front_right_chassis");
        backLeftDrive  = hardware.get(DcMotor.class, "back_left_chassis");
        backRightDrive = hardware.get(DcMotor.class, "back_right_chassis");

        DcMotor[] drives = {frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};
        for (DcMotor drive : drives) {
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        
        largeRollers = hardware.get(DcMotor.class, "large_roller");
        smallRollers = hardware.get(DcMotor.class, "small_roller");
        flywheel = hardware.get(DcMotor.class, "flywheel");

        loadingServo = hardware.get(Servo.class, "reload_servo");
        wobbleServo = hardware.get(Servo.class, "wobble_servo");

        slideDrive = hardware.get(DcMotor.class, "slide_drive");
        slideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorDistance = hardware.get(RevColorSensorV3.class, "color_distance");

        imu = hardware.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        this.imu.initialize(parameters);
        this.lastAngles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        this.globalAngle = 0;

        this.raiseWobble();
    }

    public void driveWheels(double bl, double fl, double br, double fr) {
        frontLeftDrive.setPower(Range.clip(fl,-1.0,1.0));
        backLeftDrive.setPower(Range.clip(bl,-1.0,1.0));
        frontRightDrive.setPower(Range.clip(fr,-1.0,1.0));
        backRightDrive.setPower(Range.clip(br,-1.0,1.0));
    }

    private double protect(double in) {
        if (Math.abs(in) < 0.1) {
            if (in < 0)
                return -0.1;
            else
                return 0.1;
        } else {
            return in;
        }
    }

//    public String getRingType() {
//
//    }
    public void printRingDistance(Telemetry telemetry) {
        telemetry.addData("ring distance: ",colorDistance.getDistance(DistanceUnit.INCH));
    }

    public void setProtectedPower(double bl, double fl, double br, double fr) {
        frontLeftDrive.setPower(Range.clip(protect(bl),-1.0,1.0));
        backLeftDrive.setPower(Range.clip(protect(fl),-1.0,1.0));
        frontRightDrive.setPower(Range.clip(protect(br),-1.0,1.0));
        backRightDrive.setPower(Range.clip(protect(fr),-1.0,1.0));
    }

    public double targetWaitTime = 0;

    public void strafe(double drive, double strafe, double turn, double speed, double dt)    {
        double robotAngle = getAngle()/180*Math.PI;
        targetWaitTime += dt;
        if (turn != 0) {
            targetWaitTime = 0;
        }
        if (targetWaitTime < 0.25)
            this.targetAngle = robotAngle;
        double fixTurn = 0;
        if (drive == 0 && strafe == 0 && getDifferenceBetweenAngles(robotAngle,targetAngle) > Math.PI/24) {
            int dir = 1;
            double delta = targetAngle - robotAngle;
            if (targetAngle < robotAngle)
                delta = 2 * Math.PI - robotAngle + targetAngle;
            if (delta < Math.PI)
                dir = -1;
            fixTurn = 0.7 * dir;
        }
        turn += fixTurn;
        double backLeftPower   = Range.clip((drive + turn + strafe) * speed, -1.0, 1.0);
        double frontLeftPower  = Range.clip((drive + turn - strafe) * speed, -1.0, 1.0);
        double backRightPower  = Range.clip((drive - turn - strafe) * speed, -1.0, 1.0);
        double frontRightPower = Range.clip((drive - turn + strafe) * speed, -1.0, 1.0);
        this.driveWheels(backLeftPower, frontLeftPower, backRightPower, frontRightPower);
    }

    public void changeTurnAngle(double angle, double dt) {
        changeTurnAngle(angle * dt);
    }

    public void changeTurnAngle(double angle) {
        this.targetAngle += angle;
        // restrict between [0, 2pi)
        while (this.targetAngle < 0)
            targetAngle += Math.PI * 2;
        while (this.targetAngle > 2 * Math.PI)
            targetAngle -= Math.PI * 2;
    }

    public void changeTurnAngleDegrees(double degrees) {
        changeTurnAngle(Math.toRadians(degrees));
    }

    public void driveForward(double power) {
        driveWheels(power, power, power, power);
    }

    public boolean driveIsBusy() {
        return this.backLeftDrive.isBusy() && this.backRightDrive.isBusy() && this.frontLeftDrive.isBusy() && this.frontRightDrive.isBusy();
    }

    public void stopChassis() {
        driveWheels(0,0,0,0);
    }

    public void printEncoderPositions(Telemetry telemetry) {
        telemetry.addData("left front","%7d/%7d",this.frontLeftDrive.getCurrentPosition(),this.frontLeftDrive.getTargetPosition());
        telemetry.addData("right front","%7d/%7d",this.frontRightDrive.getCurrentPosition(),this.frontRightDrive.getTargetPosition());
        telemetry.addData("left back","%7d/%7d",this.backLeftDrive.getCurrentPosition(),this.backLeftDrive.getTargetPosition());
        telemetry.addData("right back","%7d/%7d",this.backRightDrive.getCurrentPosition(),this.backRightDrive.getTargetPosition());
    }

    public void clearEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void vectorStrafe(Vector movementVector, double turn, double power, double dt) {
        movementVector.normalize();
        this.relativeStrafe(movementVector.y, movementVector.x, turn, power, dt);
    }

    public void relativeStrafe(double drive, double strafe, double turn, double power, double dt) {
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
        double robotAngle = getAngle();

        // convert degrees to radians
        robotAngle = (robotAngle / 180) * Math.PI;

        // correct for desired angle
        double alpha = angle - robotAngle;

        double mag = Math.sqrt(drive * drive + strafe * strafe);

        // apply trigonometry
        drive = mag * Math.sin(alpha);
        strafe = mag * Math.cos(alpha);

        this.strafe(drive, strafe, turn, power, dt);
    }

    public void intake(double power)   {
        power = Range.clip(power, -1.0, 1.0);
        largeRollers.setPower(power);
        smallRollers.setPower(-power);
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double deltaAngle = angles.thirdAngle - lastAngles.thirdAngle;
        if (deltaAngle < -180)
            deltaAngle+=360;
        else if (deltaAngle > 180)
            deltaAngle-=360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        while (globalAngle  < 0)
            globalAngle += 360;
        globalAngle %= 360;

        return globalAngle;
    }

    public void loadRing(boolean load, double elapsed)  {
        if(load && retractTime < elapsed && currentFlyPower != 0)    {
            loadingServo.setPosition(loadedPosition);
            retractTime = elapsed + loadingTime;
        }
        else if(retractTime < elapsed)    {
            loadingServo.setPosition(storingPosition);
        }
    }

    public void forceLoadRing() {
        loadingServo.setPosition(loadedPosition);
    }

    public void forceUnloadRing() {
        loadingServo.setPosition(storingPosition);
    }

    double currentFlyPower = 0;
    public void setFlywheelPower(double power)  {
        flywheel.setPower(power);
        currentFlyPower = power;
    }

    public void toggleWobble() {
        if (wobbleServo.getPosition() > 0.5)
            raiseWobble();
        else
            lowerWobble();
    }

    public void lowerWobble() {
        wobbleServo.setPosition(0.0);
    }

    public void raiseWobble() {
        wobbleServo.setPosition(0.85);
    }

    public void setSlidePower(double power) {
        slideDrive.setPower(power);
    }

    /**
     * Returns the smallest difference between the two angles
     * Assumes both angles between [0,2pi)
     * @param angle1
     * @param angle2
     * @return
     */
    private double getDifferenceBetweenAngles(double angle1, double angle2) {
        // thrm 1. a dot b = |a||b|cos(theta)
        //         cos(a1)*cos(b1)+sin(a2)*sin(b2) = cos(theta)
        //         theta = arccos(cos(a1)*cos(a2)+sin(a1)*sin(a2))

        double theta = Math.acos(Math.cos(angle1)*Math.cos(angle2)+Math.sin(angle1)*Math.sin(angle2));
        return theta;
    }
}
