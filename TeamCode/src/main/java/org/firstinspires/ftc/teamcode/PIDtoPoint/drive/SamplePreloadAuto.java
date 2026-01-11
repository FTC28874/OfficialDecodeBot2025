package org.firstinspires.ftc.teamcode.PIDtoPoint.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

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
        DRIVE_START_TO_SHOOT,
        PAUSE_AFTER_START,
        PAUSE_FOR_SHOOT_1,

        DRIVE_SHOOT_TO_INTAKE_READY_2,
        PAUSE_AFTER_INTAKE_READY_2,
        DRIVE_INTAKE_READY_2_TO_INTAKE_2,
        PAUSE_AFTER_INTAKE_2,
        DRIVEBACK_TO_INTAKE_READY_2,
//asdfs
        DRIVE_INTAKE_2_TO_READY_TO_EMPTY,
        PAUSE_AFTER_READY_TO_EMPTY,
        DRIVE_READY_TO_EMPTY_TO_GATE,
        PAUSE_AFTER_GATE,
        DRIVE_GATE_TO_SHOOT_LINE,
        PAUSE_AFTER_SHOOT_LINE,
        PAUSE_FOR_SHOOT_2,

        DRIVE_SHOOT_TO_INTAKE_READY_1,
        PAUSE_AFTER_INTAKE_READY_1,
        DRIVE_INTAKE_READY_1_TO_INTAKE_1,
        PAUSE_AFTER_INTAKE_1,
        DRIVE_INTAKE_1_TO_SHOOT,
        PAUSE_FOR_SHOOT_3,

        DRIVE_SHOOT_TO_INTAKE_READY_3,
        PAUSE_AFTER_INTAKE_READY_3,
        DRIVE_INTAKE_READY_3_TO_INTAKE_3,
        PAUSE_AFTER_INTAKE_3,
        DRIVE_INTAKE_3_TO_SHOOT,
        PAUSE_FOR_SHOOT_4,

        DRIVE_SHOOT_TO_READY_TO_EMPTY_END,
        DONE
    }
    static Pose2D pose(double xIn, double yIn, double headingDeg) {
        return new Pose2D(
                DistanceUnit.MM,
                xIn * 25.4,
                yIn * 25.4,
                AngleUnit.DEGREES,
                headingDeg
        );
    }
    static final Pose2D START_POSE = pose(88, 9, 90);
    static final Pose2D SHOOT_POSE = pose(88, 95, 0);
   static final Pose2D READY_TO_EMPTY = pose(110, 66, 90);
   static final Pose2D EMPTY_GATE = pose(120, 66, 90);
    static final Pose2D SHOOTING_LINE = pose(88, 66, 90);
    static final Pose2D READY_TO_EMPTY_END = pose(110, 66, 90);

    static final Pose2D INTAKE_READY_SET_1 = pose(88, 84, 0);
    static final Pose2D ACTUALLY_DO_INTAKE_SET_1 = pose(111, 84, 0);
    static final Pose2D INTAKE_READY_SET_2 = pose(88, 60, 0);
    static final Pose2D ACTUALLY_DO_INTAKE_SET_2 = pose(111, 60, 0);

    static final Pose2D INTAKE_READY_SET_3 = pose(88, 36, 0);
    static final Pose2D ACTUALLY_DO_INTAKE_SET_3 = pose(111, 36, 0);



    @Override
    public void runOpMode() {

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
        odo.setOffsets(-35.8, -156.35); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.DRIVE_START_TO_SHOOT;



        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            odo.setPosition(START_POSE);

            switch (stateMachine){
                case DRIVE_START_TO_SHOOT:
                    //the first step in the autonomous
                    stateMachine = StateMachine.PAUSE_AFTER_START;
                    break;
                case PAUSE_AFTER_START:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */

                    if (nav.driveTo(odo.getPosition(), SHOOT_POSE, 0.7, 0)){
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.PAUSE_FOR_SHOOT_1;
                    }
                    break;
//                case PAUSE_FOR_SHOOT_1:
//                    //drive to the second target
//                    if (timer.time() > 1) {
//                        stateMachine = StateMachine.DRIVE_SHOOT_TO_INTAKE_READY_2;
//                    }
//                    break;
//                case DRIVE_SHOOT_TO_INTAKE_READY_2:
//                    if (nav.driveTo(odo.getPosition(), INTAKE_READY_SET_2, 0.7, 0)){
//                        telemetry.addLine("at position #1!");
//                        stateMachine = StateMachine.PAUSE_AFTER_INTAKE_READY_2;
//                    }
//                    break;
//                case PAUSE_AFTER_INTAKE_READY_2:
//                    if (timer.time() > 1) {
//                        stateMachine = StateMachine.DRIVE_INTAKE_READY_2_TO_INTAKE_2;
//                    }
//                    break;
//                case DRIVE_INTAKE_READY_2_TO_INTAKE_2:
//                    if (nav.driveTo(odo.getPosition(), ACTUALLY_DO_INTAKE_SET_2, 0.7, 0)){
//                        telemetry.addLine("at position #1!");
//                        stateMachine = StateMachine.PAUSE_AFTER_INTAKE_2;
//                    }
//                    break;
//                case PAUSE_AFTER_INTAKE_2:
//                    if (timer.time() > 1) {
//                        stateMachine = StateMachine.DRIVEBACK_TO_INTAKE_READY_2;
//                    }
//                    break;
//                case DRIVEBACK_TO_INTAKE_READY_2:
//                    if (nav.driveTo(odo.getPosition(), INTAKE_READY_SET_2, 0.7, 0)){
//                        telemetry.addLine("at position #1!");
//                        stateMachine = StateMachine.DRIVE_INTAKE_2_TO_READY_TO_EMPTY;
//                    }
//                    break;
//                case DRIVE_INTAKE_2_TO_READY_TO_EMPTY:
//                    if (nav.driveTo(odo.getPosition(),READY_TO_EMPTY, 0.7, 0)){
//                        telemetry.addLine("at position #1!");
//                        stateMachine = StateMachine.PAUSE_AFTER_READY_TO_EMPTY;
//                    }
//                    break;
//////                case DRIVE_TO_TARGET_4:
//////                    if(nav.driveTo(odo.getPosition(),TARGET_4,0.7,1)){
//////                        telemetry.addLine("at position #4");
//////                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
//////                    }
//////                    break;
//////                case DRIVE_TO_TARGET_5:
//////                    if(nav.driveTo(odo.getPosition(),TARGET_5,0.7,1)){
//////                        telemetry.addLine("There!");
//////                        stateMachine = StateMachine.AT_TARGET;
//////                    }
////                    break;
            }


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
