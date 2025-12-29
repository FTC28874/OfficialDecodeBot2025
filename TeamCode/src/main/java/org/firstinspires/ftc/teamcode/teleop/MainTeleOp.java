package org.firstinspires.ftc.teamcode.teleop;

import androidx.appcompat.widget.ThemedSpinnerAdapter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.HelperServos;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

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

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // DT Hardware Mapping
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

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
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double powerFL  = axial + lateral + yaw;
            double powerFR = axial - lateral - yaw;
            double powerBL   = axial - lateral + yaw;
            double powerBR  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(powerFL), Math.abs(powerFR));
            max = Math.max(max, Math.abs(powerBL));
            max = Math.max(max, Math.abs(powerBR));

            if (max > 1.0) {
                powerFL  /= max;
                powerFR /= max;
                powerBL   /= max;
                powerBR  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            driveFL.setPower(powerFL);
            driveFR.setPower(powerFR);
            driveBL.setPower(powerBL);
            driveBR.setPower(powerBR);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Shooter RPM: ", Shooter.getCurrentRPM());
            telemetry.addData("Shooter Target Speed: ", shooterEncSpeed);
            telemetry.addData("Shooter RPM Error: ", Math.abs(Shooter.getCurrentRPM() - shooterEncSpeed));
            telemetry.update();

            // --- Shooter / Intake Controls ---

            // Shooter Speed Control
            if (gamepad1.dpadUpWasPressed()) {
                shooterEncSpeed = shooterEncSpeed + 50;
            }
            if (gamepad1.dpadDownWasPressed()) {
                shooterEncSpeed = shooterEncSpeed - 50;
            }

            // Intake Controls
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

            // Shooter FlyWheel Control
            if (gamepad2.right_bumper) {
                Shooter.setShooterPower(Shooter.PIDControl(shooterEncSpeed, Shooter.getCurrentRPM()));
            }
            if (!gamepad2.right_bumper) {
                Shooter.stopShooter();
            }

            // Shooter Hood Controls
            Shooter.setShooterPosition(shooterHoodAngle);

            if (gamepad2.dpadUpWasPressed()) {
                shooterHoodAngle = Shooter.HoodState.UP.angle;
            }
            if (gamepad2.dpadDownWasPressed()) {
                shooterHoodAngle = Shooter.HoodState.DOWN.angle;
            }
            if (gamepad2.dpadRightWasPressed()) {
                if (shooterHoodAngle < Shooter.HoodState.UP.angle && shooterHoodAngle >= Shooter.HoodState.DOWN.angle) {
                    shooterHoodAngle = shooterHoodAngle + 0.05;
                }
            }
            if (gamepad2.dpadLeftWasPressed()) {
                if (shooterHoodAngle <= Shooter.HoodState.UP.angle && shooterHoodAngle > Shooter.HoodState.DOWN.angle) {
                    shooterHoodAngle = shooterHoodAngle - 0.05;
                }
            }

            // Turret Control
            if (gamepad1.rightBumperWasPressed()) {
                Shooter.turnTurretDirection(true, 0.25);
                sleep(250);
                Shooter.turnTurretDirection(true, 0.0);
            }
            if (gamepad1.leftBumperWasPressed()) {
                Shooter.turnTurretDirection(false, 0.25);
                sleep(250);
                Shooter.turnTurretDirection(false, 0.0);
            }


            if (gamepad1.aWasPressed()) {
                HelperServos.setPusherRest();
            }
            if (gamepad1.bWasPressed()) {
                HelperServos.setPusherPush();
            }







        }
    }
}