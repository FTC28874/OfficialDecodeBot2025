package org.firstinspires.ftc.teamcode.PIDtoPoint.drive.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Intake;

import java.util.Locale;

// RPM: 1600
// Turret Encoder Value: 406
// Shooter Hood: 0.2

// Far:
// RPM: 1950
// Turret Encoder Value: -126
// Shooter Hood: 0.55

@Autonomous(name="Sample Preload Auto", group="Auto")
//@Disabled

public class SamplePreloadAuto extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    ElapsedTime timer = null;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        WAIT_TO_CLAW,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.INCH, 72, 0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.INCH, 45, 0, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.INCH,300,300, AngleUnit.DEGREES,-45);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.INCH, 100, -2600, AngleUnit.DEGREES, 90);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.INCH, 100, 0, AngleUnit.DEGREES, 0);

    public double inToMM(double inValue) {
        return inValue * 25.4;
    }


    @Override
    public void runOpMode() {

        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "driveFL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveFR");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "driveBL");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "driveBR");

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

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            Shooter.setShooterPower(Shooter.PIDControl(1640, Shooter.getCurrentRPM()));
            Intake.lowerIntake();
            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;
                case DRIVE_TO_TARGET_1:
                    Shooter.setShooterPosition(0.12);
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), TARGET_1, 0.7, 0.5)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.WAIT_TO_CLAW;
                    }
                    timer.reset();
                    break;
                case WAIT_TO_CLAW:
                    Intake.runIntake();
                    if (timer.time() > 1.0) {
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    Intake.lowerIntake();
                    break;
                case DRIVE_TO_TARGET_2:
                    //drive to the second target
                    Intake.stopIntake();
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 0.4, 1)) {
                        telemetry.addLine("at position #2!");
                        timer.reset();
                        stateMachine = StateMachine.WAIT_TO_CLAW;
                    }
                    break;
//                case WAIT_TO_CLAW:
//                    if (timer.time() > 0.5) {
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
//                    }
//                    break;
//                case DRIVE_TO_TARGET_3:
//                    if(nav.driveTo(odo.getPosition(), TARGET_3, 0.7, 0)){
//                        telemetry.addLine("at position #3");
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
//                    }
//                    break;
////                case DRIVE_TO_TARGET_4:
////                    if(nav.driveTo(odo.getPosition(),TARGET_4,0.7,1)){
////                        telemetry.addLine("at position #4");
////                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
////                    }
////                    break;
////                case DRIVE_TO_TARGET_5:
////                    if(nav.driveTo(odo.getPosition(),TARGET_5,0.7,1)){
////                        telemetry.addLine("There!");
////                        stateMachine = StateMachine.AT_TARGET;
////                    }
//                    break;
            }
//as

            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:",stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();

        }
    }}