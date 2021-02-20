package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;


@TeleOp(name="Main TeleOp Iterative", group="Iterative Opmode")
public class MainTeleOpIterative extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private final double driveSpeed = 1;
    private final double slowSpeed = 0.25;
    private final double flywheelSpeed = RobotHardware.DEFAULT_FLYWHEEL_POWER;

    boolean rightDpadClicked = false;
    boolean leftDpadClicked = false;
    boolean bDown = false;
    boolean startDown = false;
    boolean rightStickDown = false;

    boolean secondStageActive = true;

    double flywheelPowerDelta = 0;

    boolean useRelativeStrafe = true;

    RobotHardware robot;
    double last;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new RobotHardware(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
        last = runtime.milliseconds();
    }

    private String team = "blue";

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (this.gamepad1.x)
            team = "blue";
        else if (this.gamepad1.b)
            team = "red";
        telemetry.addData("Selected Team", team);
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        if (team.equals("blue"))
            robot.setAngleOffset(90);
        else
            robot.setAngleOffset(-90);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Drive Mode", useRelativeStrafe ? "Relative Strafe" : "Regular Strafe");
        telemetry.addData("Second Stage Active?",secondStageActive ? "YES" : "NO");
        robot.printInformation(telemetry);
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
            flywheelPowerDelta -= 0.01;
            rightDpadClicked = true;
        }
        else if (!gamepad1.dpad_right)
            rightDpadClicked = false;

        if(gamepad1.dpad_left && !leftDpadClicked)    {
            flywheelPowerDelta += 0.01;
            leftDpadClicked = true;
        }
        else if (!gamepad1.dpad_left)
            leftDpadClicked = false;

        robot.setTargetFlywheelPower(robot.getRecommendedFlywheelPower() + flywheelPowerDelta);

        double forward = 0;
        if (gamepad1.a)
            forward-=0.7;
        if (gamepad1.y)
            forward+=0.7;

        //endregion
//            robot.strafe(drive, strafe, turn, speed, dt);
        if (useRelativeStrafe) {
            if (forward == 0)
                robot.relativeStrafe(drive, strafe, turn, speed, dt);
            else
                robot.strafe(forward, 0, 0, 0.8, dt);
        } else {
            robot.strafe(drive+forward, strafe, turn, speed, dt);
        }
        if (secondStageActive)
            robot.intake(gamepad1.right_trigger - gamepad1.left_trigger);
        else
            robot.singleIntake(gamepad1.right_trigger - gamepad1.left_trigger);

        if(gamepad1.right_bumper)
            robot.setFlywheelPower(robot.getTargetFlywheelPower());
        else
            robot.setFlywheelPower(0);

        robot.loadRing(gamepad1.left_bumper, runtime.seconds());

        if (!bDown && gamepad1.b) {
            bDown = true;
            robot.toggleWobble();
        }
        else if (!gamepad1.b) bDown = false;

        if (!startDown && gamepad1.start) {
            startDown = true;
            useRelativeStrafe = !useRelativeStrafe;
        }
        else if (!gamepad1.start) startDown = false;

        if (!rightStickDown && gamepad1.right_stick_button) {
            rightStickDown = true;
            secondStageActive = !secondStageActive;
        }
        else if (!gamepad1.right_stick_button) rightStickDown = false;

        if (gamepad1.x) {
            robot.setTargetAngleDegrees(robot.getTargetShootingPosition());
        }

        robot.setSlidePower(0);
        if (gamepad1.dpad_up) {
            robot.setSlidePower(1);
        } else if (gamepad1.dpad_down) {
            robot.setSlidePower(-1);
        }

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.closeTfod();
    }

}
