package org.firstinspires.ftc.teamcode.PIDtoPoint.drive.auto;

import androidx.appcompat.widget.ThemedSpinnerAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.HelperServos;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;

import java.util.Locale;

// RPM: 1800
// Turret Encoder Value: 406
// Shooter Hood: 0.2

// Far:
// RPM: 1950
// Turret Encoder Value: 126
// Shooter Hood: 0.55

@Autonomous(name="Close To Red Goal", group="Auto", preselectTeleOp = "New Main TeleOp")
//@Disabled

public class CloseToRedGoal extends LinearOpMode {

    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;
    DcMotorEx turretEncoder = null;
    DcMotor turret = null;
    ElapsedTime timer = null; //asdf
    double shooterEncSpeed = 2000;
    double shooterHoodAngle = Shooter.ShootPositionState.MID_MID_HOOD.position;
    int ballMode = 0;
    static double localKpPosition = 0.005;
    static double localKpHeading = 0.01;

    GoBildaPinpointDriver odo = null; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        WAIT,
        WAIT_2,
        WAIT_3,
        WAIT_4,
        WAIT_5,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8,
        DRIVE_TO_TARGET_9,
        DRIVE_TO_TARGET_10,
        DONE
    }

    static final Pose2D BEGIN_INTAKE_ROW_2 = new Pose2D(DistanceUnit.INCH, -63, 25,AngleUnit.DEGREES,-90);
    static final Pose2D SHOOT_POSE = new Pose2D(DistanceUnit.INCH, -24, 24, AngleUnit.DEGREES, -45);
    static final Pose2D INTAKE_ROW_2 = new Pose2D(DistanceUnit.INCH, -63, 1, AngleUnit.DEGREES, -90);
    static final Pose2D BEGIN_INTAKE_ROW_1 = new Pose2D(DistanceUnit.INCH,-37,25, AngleUnit.DEGREES,-90);
    static final Pose2D INTAKE_ROW_1 = new Pose2D(DistanceUnit.INCH, -37, 1, AngleUnit.DEGREES, -90);
    static final Pose2D BEGIN_INTAKE_ROW_3 = new Pose2D(DistanceUnit.INCH, -84, 25, AngleUnit.DEGREES, -90);
    static final Pose2D INTAKE_ROW_3 = new Pose2D(DistanceUnit.INCH, -84, 1, AngleUnit.DEGREES, -90);
    static final Pose2D END_AUTO = new Pose2D(DistanceUnit.INCH, -49, 12, AngleUnit.DEGREES, -90);


    public double inToMM(double inValue) {
        return inValue * 25.4;
    }


    @Override
    public void runOpMode() {

        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);
        HelperServos.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "driveFL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveFR");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "driveBL");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "driveBR");

        turret = hardwareMap.get(DcMotor.class, "turret");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "driveFR");

        timer = new ElapsedTime();

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-48, -156); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();



        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        Intake.lowerIntake();
        sleep(1500);
        Shooter.lowerShooter();
        sleep(1500);
        Shooter.raiseShooter();
        sleep(1500);
        Shooter.setShooterPosition(0.55);
        HelperServos.setStopperPass();

        while (opModeInInit()) {
            telemetry.addLine("Select [A] for 3 Ball Auto");
            telemetry.addLine("Select [B] for 6 Ball Auto");
            telemetry.addLine("Select [Y] for 9 Ball Auto");
            telemetry.addLine("Select [X] for 9 Ball Auto With 3rd Row");
            telemetry.addData("Current Auto Mode", ballMode);
            telemetry.addData("Current kP_Position", localKpPosition);
            telemetry.addData("Current kP_Heading", localKpHeading);
            if (gamepad1.aWasPressed()) {
                ballMode = 3;
            }
            if (gamepad1.bWasPressed()) {
                ballMode = 6;
            }
            if (gamepad1.xWasPressed()) {
                ballMode = 12;
            }
            if (gamepad1.yWasPressed()) {
                ballMode = 9;
            }
            if (gamepad1.dpadUpWasPressed()) {
                localKpPosition += 0.001;
            }
            if (gamepad1.dpadDownWasPressed()) {
                localKpPosition -= 0.001;
            }
            if (gamepad1.dpadRightWasPressed()) {
                localKpHeading += 0.001;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                localKpHeading -= 0.001;
            }
            MecanumDrive.changeKp(localKpPosition, localKpHeading);
            telemetry.update();
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            Shooter.setShooterPosition(shooterHoodAngle);
            Shooter.setShooterPower(Shooter.PIDControl(shooterEncSpeed, Shooter.getCurrentRPM()));
            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    timer.reset();
                    stateMachine = StateMachine.WAIT;
                    break;
                case WAIT:
                    if (timer.time() > 1.0) {
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    }
                    break;
                case DRIVE_TO_TARGET_1:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), SHOOT_POSE, 0.25, 0.0)) {
                        Intake.raiseIntake();
                        telemetry.addLine("about to intake with stopper closed");
                        telemetry.addLine("at position #1!");
                        timer.reset();
                        stateMachine = StateMachine.WAIT_2;
                    }
                    break;
                case WAIT_2:
                    if (timer.time() > 0.5) {
                        Intake.runIntake();
                    }
                    if (timer.time() > 2.5) {
                        timer.reset();
                        HelperServos.setStopperStop();
                        if (ballMode > 3) {
                            stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                        } else {
                            stateMachine = StateMachine.DONE;
                        }
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), BEGIN_INTAKE_ROW_1, 0.5, 0.5)) {
                        telemetry.addLine("at position #2!");
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if (nav.driveTo(odo.getPosition(), INTAKE_ROW_1, 0.5, 0.0)) {
                        telemetry.addLine("At position #3");
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;

                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if (nav.driveTo(odo.getPosition(), SHOOT_POSE, 0.5, 0.5)) {
                        timer.reset();
                        stateMachine = StateMachine.WAIT_3;
                    }
                    break;
                case WAIT_3:
                    if (timer.time() > 0.5) {
                        HelperServos.setStopperPass();
                    }
                    if (timer.time() > 1.0) {
                        Intake.lowerIntake();
                    }
                    if (timer.time() > 2.5) {
                        timer.reset();
                        Intake.raiseIntake();
                        HelperServos.setStopperStop();
                        if (ballMode > 6) {
                            stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                        } else {
                            stateMachine = StateMachine.DONE;
                        }
                    }
                    break;
                    // loop from here
                case DRIVE_TO_TARGET_5:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), BEGIN_INTAKE_ROW_2, 0.5, 0.5)) {
                        telemetry.addLine("at position #2!");
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if (nav.driveTo(odo.getPosition(), INTAKE_ROW_2, 0.5, 0.0)) {
                        telemetry.addLine("At position #3");
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;

                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if (nav.driveTo(odo.getPosition(), SHOOT_POSE, 0.5, 0.5)) {
                        timer.reset();
                        stateMachine = StateMachine.WAIT_4;
                    }
                    break;
                case WAIT_4:
                    if (timer.time() > 0.5) {
                        HelperServos.setStopperPass();
                    }
                    if (timer.time() > 1.0) {
                        Intake.lowerIntake();
                    }
                    if (timer.time() > 2.5) {
                        timer.reset();
                        Intake.raiseIntake();
                        HelperServos.setStopperStop();
                        if (ballMode > 9) {
                            stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                        } else {
                            stateMachine = StateMachine.DONE;
                        }
                    }
                    break;
                    // to here
                case DRIVE_TO_TARGET_8:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), BEGIN_INTAKE_ROW_3, 0.5, 0.5)) {
                        telemetry.addLine("at position #2!");
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if (nav.driveTo(odo.getPosition(), INTAKE_ROW_3, 0.5, 0.0)) {
                        telemetry.addLine("At position #3");
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;

                    }
                    break;
                case DRIVE_TO_TARGET_10:
                    if (nav.driveTo(odo.getPosition(), BEGIN_INTAKE_ROW_3, 0.5, 0.0)) {
                        timer.reset();
                        stateMachine = StateMachine.DONE;
                    }
                    break;
                case DONE:
                    if (nav.driveTo(odo.getPosition(), END_AUTO, 0.7, 0.5)) {
                        timer.reset();
                        shooterEncSpeed = 0;
                        Intake.stopIntake();
                    }
                    break;
            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:", stateMachine);
            telemetry.addData("time:", timer.time());

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();

        }
    }}