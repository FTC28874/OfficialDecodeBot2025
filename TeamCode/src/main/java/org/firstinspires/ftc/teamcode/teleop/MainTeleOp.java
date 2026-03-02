package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.DynamicShooter;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.HelperServos;
import org.firstinspires.ftc.teamcode.robot.Turret;


// speed close 1200
// speed far pos 2000

@TeleOp(name="Main Teleop", group="Linear OpMode")
//@Disabled
public class MainTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveFL = null;
    private DcMotor driveBL = null;
    private DcMotor driveFR = null;
    private DcMotor driveBR = null;

    private GoBildaPinpointDriver odo = null;

    private double shooterEncSpeed = 1600;
    private double shooterHoodAngle = Shooter.HoodState.DOWN.angle;

    // All values in INCHES, then converted to mm if using mm for odometry

    final double startX_in = +63.0;   // robot center X in inches
    final double startY_in = +60.0;   // robot center Y in inches
    final double startHeading_deg = 0.0; // facing upfield (+Y)

    // Convert to millimeters for most odometry systems:
    final double startX_mm = startX_in * 25.4;
    final double startY_mm = startY_in * 25.4;

    private boolean dynamicToggle = false;
    private boolean prevB = false;
    private double driverSensitivity = 1.0;

    Pose2D pos = null;
    private double shooterPower = 0;
    private double robotHeading = 0;
    private boolean isDynamic = true;
    private double goalDistance = 0;
    private double turretPos = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // DT Hardware Mapping
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init Helper Classes
        org.firstinspires.ftc.teamcode.robot.Shooter.init(hardwareMap);
        org.firstinspires.ftc.teamcode.robot.Intake.init(hardwareMap);
        HelperServos.init(hardwareMap);
        Turret.init(hardwareMap);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Reset servos
        odo.resetPosAndIMU();

//        odo.setPosX(startX_in, DistanceUnit.INCH);
//        odo.setPosY(startY_in, DistanceUnit.INCH);
//        odo.setHeading(startHeading_deg, AngleUnit.DEGREES);

        waitForStart();
        runtime.reset();

        Turret.zeroTurret();
        telemetry.addLine("Press [B] for Blue goal");
        telemetry.addLine("Press [A] for Red goal");
        while (opModeInInit()) {
            if (gamepad1.bWasPressed()) {
                DynamicShooter.setGoalPosition(-128, 128);
                telemetry.addLine("Blue goal. Ready to start");
            }
            if (gamepad1.aWasPressed()) {
                DynamicShooter.setGoalPosition(128, 128);
                telemetry.addLine("Red goal. Ready to start");
            }
        }
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y * driverSensitivity;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * driverSensitivity;
            double yaw     =  gamepad1.right_stick_x * driverSensitivity;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double powerFL = axial + lateral + yaw;
            double powerFR = axial - lateral - yaw;
            double powerBL = axial - lateral + yaw;
            double powerBR = axial + lateral - yaw;

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
            telemetry.addData("Turret Position; ", Shooter.getTurretPosition());
            telemetry.addData("ShooterServo Position: ", shooterHoodAngle);
            telemetry.addData("Dynamic Toggle: ", dynamicToggle);
            telemetry.update();


            odo.update();
            pos = odo.getPosition();
            robotHeading = pos.getHeading(AngleUnit.DEGREES);

            double curX = pos.getX(DistanceUnit.INCH);
            double curY = pos.getY(DistanceUnit.INCH);
            goalDistance = DynamicShooter.calcDistToGoal(curX, curY);

            // --- Shooter / Intake Controls ---

            // Shooter Speed Control    ```````
            if (gamepad1.rightBumperWasPressed()) {
                shooterEncSpeed = shooterEncSpeed + 50;
            }
            if (gamepad1.leftBumperWasPressed()) {
                shooterEncSpeed = shooterEncSpeed - 50;
            }
            if (gamepad1.dpadUpWasPressed()) {
                shooterEncSpeed = 3250;
                shooterHoodAngle = 0.55;
            }
            if (gamepad1.dpadDownWasPressed()) {
                shooterEncSpeed = 1800;
                shooterHoodAngle = Shooter.HoodState.DOWN.angle;
            }
            if (gamepad1.dpadRightWasPressed()) {
                shooterEncSpeed = 2450;
                shooterHoodAngle = 0.3;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                shooterEncSpeed = 2000;
                shooterHoodAngle = 0.25;
            }

            // Driver Sensitivity Controls
            if (gamepad1.yWasPressed()) {
                shooterEncSpeed = 0;
                driverSensitivity = 0.5;
            }
            if (gamepad1.xWasPressed()) {
                shooterEncSpeed = 1200;
                driverSensitivity = 1.0;
            }



            // Intake Controls
            if (gamepad2.left_bumper && !gamepad2.a && !gamepad2.x) {
                Intake.runIntake();
                HelperServos.setStopperStop();
                Intake.raiseIntake();
            }
            if (gamepad2.a && gamepad2.left_bumper && !gamepad2.x) {
                Intake.reverseIntake();
                Intake.raiseIntake();
            }
            if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.x) {
                Intake.stopIntake();
            }
            if (gamepad2.x && !gamepad2.a && !gamepad2.left_bumper) {
                Intake.runIntake();
                HelperServos.setStopperPass();
            }

            if (gamepad2.yWasPressed()) {
                Intake.lowerIntake();
            }

            // Shooter FlyWheel Control
            if (!gamepad2.right_bumper) {
                Shooter.setShooterPower(Shooter.PIDControl(shooterEncSpeed, Shooter.getCurrentRPM()));
            }
            if (gamepad2.right_bumper) {
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

//            // Turret Control
//            if (gamepad1.rightBumperWasPressed()) {
//                Shooter.turnTurretDirection(true, 0.25);
//                sleep(250);
//                Shooter.turnTurretDirection(true, 0.0);
//            }
//            if (gamepad1.leftBumperWasPressed()) {
//                Shooter.turnTurretDirection(false, 0.25);
//                sleep(250);
//                Shooter.turnTurretDirection(false, 0.0);
//            }

            if (gamepad1.a && gamepad1.left_bumper) {
                odo.resetPosAndIMU();
                DynamicShooter.resetTurretEncoder();
            }

            // Dynamic Toggle
            if (gamepad1.bWasPressed()) {
                isDynamic = !isDynamic;
            }

            if (isDynamic) {
                shooterEncSpeed = DynamicShooter.calcTargetRPM(goalDistance);
                shooterHoodAngle = DynamicShooter.calcHoodPos(goalDistance);

                turretPos = DynamicShooter.calcTurretHead(robotHeading, curX, curY, true);
                DynamicShooter.setTurretToPosition( (int) turretPos, 0.85);

            }

            if (gamepad2.left_trigger > 0.1) {
                if (Shooter.getTurretPosition() > Constants.MIN_TURRET_HEAD) {
                    Shooter.turnTurretDirection(false, gamepad2.left_trigger * 0.6);
                }
            }
            if (gamepad2.right_trigger > 0.1) {
                if (Shooter.getTurretPosition() > Constants.MAX_TURRET_HEAD) {
                    Shooter.turnTurretDirection(true, gamepad2.right_trigger * 0.6);
                }
            }
            if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
                Shooter.turnTurretDirection(true, 0);
            }
        }
    }
}