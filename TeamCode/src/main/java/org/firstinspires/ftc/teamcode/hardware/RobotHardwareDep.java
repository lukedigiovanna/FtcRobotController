package org.firstinspires.ftc.teamcode.hardware;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Map;
import java.util.HashMap;

/**
 * Holds all motors, sensors, servos, etc.
 * Does not provide any implementation for the function of these.
 */
@Deprecated
public class RobotHardwareDep {
    private static RobotHardwareDep mainRobot;
    public static RobotHardwareDep getMainRobotHardware(HardwareMap hardwareMap) {
        if (mainRobot == null) { // then initialize it
            mainRobot = new RobotHardwareDep();
            DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left_chassis");
            DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right_chassis");
            DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left_chassis");
            DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right_chassis");

            DcMotor largeRollers = hardwareMap.get(DcMotor.class, "large_roller");
//            DcMotor smallRollers = hardwareMap.get(DcMotor.class, "small_rollers");
//            DcMotor flywheel = hardwareMap.get(DcMotor.class, "flywheel");
//            DcMotor liftMotor = hardwareMap.get(DcMotor.class, "lift");
//            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mainRobot.put("back_left_chassis", backLeft);
            mainRobot.put("back_right_chassis", backRight);
            mainRobot.put("front_left_chassis", frontLeft);
            mainRobot.put("front_right_chassis", frontRight);
//            mainRobot.put("lift", liftMotor);
            mainRobot.put("large_roller", largeRollers);
//            mainRobot.put("small_rollers", smallRollers);
//            mainRobot.put("flywheel", flywheel);

            BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");            
            mainRobot.initIMU(imu);
        }
        return mainRobot;
    }

    public static double
            TICKS_PER_REV = 537.6,
            GEAR_REDUCTION = 1.0,
            WHEEL_DIAMETER_INCHES = 3.858,
            TICKS_PER_INCH = (TICKS_PER_REV * GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);

    // use maps to link names to motors, sensors, servos

    public void printEncoderPositions(Telemetry telemetry) {
        telemetry.addData("left front","%7d/%7d",this.motors.get("front_left_chassis").getCurrentPosition(),this.motors.get("front_left_chassis").getTargetPosition());
        telemetry.addData("right front","%7d/%7d",this.motors.get("front_right_chassis").getCurrentPosition(),this.motors.get("front_right_chassis").getTargetPosition());
        telemetry.addData("left back","%7d/%7d",this.motors.get("back_left_chassis").getCurrentPosition(),this.motors.get("back_left_chassis").getTargetPosition());
        telemetry.addData("right back","%7d/%7d",this.motors.get("back_right_chassis").getCurrentPosition(),this.motors.get("back_right_chassis").getTargetPosition());
    }
    private Map<String, DcMotor> motors;
    private Map<String, Servo> servos;
    private Map<String, Sensor> sensors;

    // IMU
    private BNO055IMU imu;
    private Orientation lastAngles;
    private double globalAngle;

    public RobotHardwareDep() {
        this.motors = new HashMap<String, DcMotor>();
        this.servos = new HashMap<String, Servo>();
        this.sensors = new HashMap<String, Sensor>();

        this.lastAngles = new Orientation();
    }

    public void initIMU(BNO055IMU imu) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        this.imu = imu;
        this.imu.initialize(parameters);
        // this.lastAngles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//        this.globalAngle = 0;
    }

    public double getAngle() {
       // if (true) return 0;

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

    public void put(String name, DcMotor motor) {
        motors.put(name, motor);
    }

    public void put(String name, Servo servo) {
        servos.put(name, servo);
    }

    public void put(String name, Sensor sensor) {
        sensors.put(name, sensor);
    }

    public DcMotor getMotor(String name) {
        return motors.get(name);
    }

    public Servo getServo(String name) {
        return servos.get(name);
    }

    public Sensor getSensor(String name) {
        return sensors.get(name);
    }
}
