package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main tele",group="Linear Opmode")
public class MainTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
     
    private final double driveSpeed = 1;
    private final double slowSpeed = 0.25;
    private final double flywheelSpeed = -0.75;

    boolean rightDpadClicked = false;
    boolean leftDpadClicked = false;

    private static final String VUFORIA_KEY =
            "Ac2AtL3/////AAABmYlEA5vrzUAniNdkaz817jg5a+xF1YvwBoaBFx+vfaoen8b7Lc/Paztzjnd1sdgsk6wIkrl5cNjZPf06xjN3DiUFwor9Va5+Dvl+JgYtaPWnCyKshUYEyIfXPnuYZQHlx2PNAp19T7QLPo1+Ks/GLK6Hi+z9bkt7Xdj91Ts9cw3U0NNK70YkR8RqakswKy673hKimeZblke+pKR94EOIQs+V99sua2mAcfBliwjSZxlyP7FCI+55kaCVYMi7+qYEmVl4D4NGi86VDyQzyG5rBYvOgaE4v8PXNSmkQH1BlV88WdOXEgvRvWmQoOMpFZ28bsuEmL6vz8fa9m4gdABsyO8AYPIOz6Lh+xalVXMmMln4";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double last = runtime.milliseconds();
        while (opModeIsActive()) {
            telemetry.addData("Robot Angle: ",robot.getAngle());
            double now = runtime.milliseconds();
            double dt = (now - last) / 1000.0;
            last = now;

            double drive    = -gamepad1.left_stick_y;
            double turn     = gamepad1.right_stick_x;
            double strafe   = gamepad1.left_stick_x;
            double speed;
            if(gamepad1.left_stick_button)
                speed = slowSpeed;
            else
                speed = driveSpeed;

            //region 90 degree turns

            if(gamepad1.dpad_right && !rightDpadClicked)    {
                robot.changeTurnAngle(-Math.PI/2);
                rightDpadClicked = true;
            }
            else if (!gamepad1.dpad_right)
                rightDpadClicked = false;

            if(gamepad1.dpad_left && !leftDpadClicked)    {
                robot.changeTurnAngle(Math.PI/2);
                leftDpadClicked = true;
            }
            else if (!gamepad1.dpad_left)
                leftDpadClicked = false;

            //endregion
            robot.relativeStrafe(drive, strafe, turn, speed, dt);
            robot.intake(gamepad1.right_trigger - gamepad1.left_trigger);

            if(gamepad1.right_bumper)
                robot.setFlywheelPower(flywheelSpeed);
            else
                robot.setFlywheelPower(0);

            robot.loadRing(gamepad1.left_bumper, runtime.seconds());

            telemetry.addData("fuck",robot.targetWaitTime);

            telemetry.update();
        }
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
