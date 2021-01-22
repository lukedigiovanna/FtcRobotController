package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

@TeleOp(name="Test", group="Linear Opmode")
public class TestTele extends LinearOpMode {

    // Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftDrive;
    DcMotor backLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backRightDrive;
    DcMotor largeRoller;
    DcMotor smallRoller;

    double driveSpeed = 1;
    double slowSpeed = 0.25;

    DcMotor flywheel;
    double flywheelSpeed = -0.8;
    double flywheelAdjustmentSensitivity = 0.05;
    boolean pressed = false;

    Servo loadingServo;
    double loadingTime = 3;
    double loadingPosition = 1;
    double storingPosition = 0;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_chassis");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_chassis");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_chassis");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_chassis");
        largeRoller = hardwareMap.get(DcMotor.class, "large_roller");
        smallRoller = hardwareMap.get(DcMotor.class, "small_roller");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        
        loadingServo = hardwareMap.get(Servo.class, "reload_servo");
        double retractTime = 0;
        double loadingTime = 2;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        largeRoller.setDirection(DcMotor.Direction.REVERSE);
        smallRoller.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive    = -gamepad1.left_stick_y;
            double turn     = gamepad1.right_stick_x;
            double strafe   = gamepad1.left_stick_x;
            double speed;
            if(gamepad1.left_stick_button)
                speed = driveSpeed;
            else
                speed = slowSpeed;

            frontLeftPower  = Range.clip((drive + turn + strafe) * speed, -1.0, 1.0);
            frontRightPower = Range.clip((drive - turn + strafe) * speed, -1.0, 1.0);
            backLeftPower   = Range.clip((drive + turn - strafe) * speed, -1.0, 1.0);
            backRightPower  = Range.clip((drive - turn - strafe) * speed, -1.0, 1.0);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
            largeRoller.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            smallRoller.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            if(gamepad1.right_bumper)
                flywheel.setPower(flywheelSpeed);
            else
                flywheel.setPower(0);
                
            if(gamepad1.left_bumper)    {
                loadingServo.setPosition(loadingPosition);
                retractTime = runtime.seconds() + loadingTime;
            }
            else if(retractTime < runtime.seconds())    {
                loadingServo.setPosition(storingPosition);
            }
            telemetry.addData("Flywheel speed", flywheelSpeed);
            telemetry.update();
        }
    }

}