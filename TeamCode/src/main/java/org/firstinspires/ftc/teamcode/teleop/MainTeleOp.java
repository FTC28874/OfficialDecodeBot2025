package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose; // ADDED: Necessary for position tracking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Constants;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.HelperServos;

@TeleOp(name="Main Teleop", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveFL = null;
    private DcMotor driveBL = null;
    private DcMotor driveFR = null;
    private DcMotor driveBR = null;

    private double shooterEncSpeed = 1600;
    private double shooterHoodAngle = Shooter.HoodState.DOWN.angle;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables.
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        // INITIALIZE FOLLOWER HERE TO PREVENT NULL ERRORS
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(105, 65, Math.toRadians(-10)));

        // Init Helper Classes
        org.firstinspires.ftc.teamcode.robot.Shooter.init(hardwareMap);
        org.firstinspires.ftc.teamcode.robot.Intake.init(hardwareMap);
        HelperServos.init(hardwareMap);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Reset servos
        Intake.raiseIntake();
        sleep(1500);
        Intake.lowerIntake();
        sleep(1500);
        Shooter.lowerShooter();
        sleep(1500);
        Shooter.raiseShooter();
        sleep(1500);
        Shooter.lowerShooter();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // CRITICAL: Update the follower to read the odometry pods
            follower.update();

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double powerFL  = axial + lateral + yaw;
            double powerFR = axial - lateral - yaw;
            double powerBL   = axial - lateral + yaw;
            double powerBR  = axial + lateral - yaw;

            max = Math.max(Math.abs(powerFL), Math.abs(powerFR));
            max = Math.max(max, Math.abs(powerBL));
            max = Math.max(max, Math.abs(powerBR));

            if (max > 1.0) {
                powerFL  /= max;
                powerFR /= max;
                powerBL   /= max;
                powerBR  /= max;
            }

            // Send calculated power to wheels
            driveFL.setPower(powerFL);
            driveFR.setPower(powerFR);
            driveBL.setPower(powerBL);
            driveBR.setPower(powerBR);

            // Show Odometry and Shooter Data
            telemetry.addData("X Position", follower.getPose().getX());
            telemetry.addData("Y Position", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Shooter RPM: ", Shooter.getCurrentRPM());
            telemetry.addData("Shooter Target Speed: ", shooterEncSpeed);
            telemetry.addData("Shooter RPM Error: ", Math.abs(Shooter.getCurrentRPM() - shooterEncSpeed));
            telemetry.update();

            // --- Shooter / Intake Controls ---
            if (gamepad1.dpad_up) { // Assuming standard dpad usage
                shooterEncSpeed = shooterEncSpeed + 50;
            }
            if (gamepad1.dpad_down) {
                shooterEncSpeed = shooterEncSpeed - 50;
            }

            if (gamepad2.left_bumper && !gamepad2.a) {
                Intake.runIntake();
                Intake.raiseIntake();
            }
            if (gamepad2.a && !gamepad2.left_bumper) {
                Intake.reverseIntake();
            }
            if (gamepad2.a && gamepad2.left_bumper) {
                Intake.reverseIntake();
                Intake.raiseIntake();
            }
            if (!gamepad2.a && !gamepad2.left_bumper) {
                Intake.stopIntake();
                Intake.lowerIntake();
            }

            if (gamepad2.right_bumper) {
                Shooter.setShooterPower(Shooter.PIDControl(shooterEncSpeed, Shooter.getCurrentRPM()));
            }
            if (!gamepad2.right_bumper) {
                Shooter.stopShooter();
            }

            Shooter.setShooterPosition(shooterHoodAngle);

            if (gamepad2.dpad_up) {
                shooterHoodAngle = Shooter.HoodState.UP.angle;
            }
            if (gamepad2.dpad_down) {
                shooterHoodAngle = Shooter.HoodState.DOWN.angle;
            }
        }
    }
}
