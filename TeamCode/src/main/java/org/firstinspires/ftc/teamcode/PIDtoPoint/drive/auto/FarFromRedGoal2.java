package org.firstinspires.ftc.teamcode.PIDtoPoint.drive.auto;




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
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.HelperServos;
import org.firstinspires.ftc.teamcode.robot.Turret;

import java.util.Locale;


// turret encoder value: -372
// hood: 0.6
// rpm: 3500

@Autonomous(name="Far From Red Goal 2", group="Auto")
//@Disabled

public class FarFromRedGoal2 extends LinearOpMode {
    private double turretPos = 0;

    DcMotor leftFrontDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightBackDrive = null;
    DcMotorEx turretEncoder = null;
    DcMotor turret = null;
    ElapsedTime timer = null; //asdf
    double shooterEncSpeed = Shooter.ShootPositionState.FAR_RPM.position;
    double shooterHoodSpeed = Shooter.ShootPositionState.FAR_HOOD.position;

    GoBildaPinpointDriver odo = null; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_HUMAN_PLAYER,
        DRIVE_TO_TARGET_2,
        SHOOT_WAIT_1,
        SHOOT_WAIT_2,
        WAIT_3,
        WAIT_4,
        DRIVE_TO_SHOOT_POS,
        BACK_TO_HUMAN_PLAYER,
        DRIVE_TO_SHOOT_POS_2,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8,
        DRIVE_TO_TARGET_9,
        DONE
    }


    static final Pose2D SHOOT_POSE = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D BEGIN_INTAKE_ROW_3 = new Pose2D(DistanceUnit.INCH, 33, -4, AngleUnit.DEGREES, -90);
    static final Pose2D INTAKE_ROW_3 = new Pose2D(DistanceUnit.INCH, 33, -4, AngleUnit.DEGREES, -90);
    static final Pose2D INTAKE_HUMAN = new Pose2D(DistanceUnit.INCH, 51, 0, AngleUnit.DEGREES, 0);




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
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        Intake.raiseIntake();
        sleep(1500);
        Intake.raiseIntake();
        sleep(1500);
        Shooter.lowerShooter();
        sleep(1500);
        Shooter.raiseShooter();
        sleep(1500);
        Shooter.setShooterPosition(0.55);
        HelperServos.setStopperPass();

        while (opModeInInit()){
            turretPos = -381;
            double power = Turret.turretPIDControl(turretPos, turret.getCurrentPosition());
            turret.setPower(power);
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            Shooter.setShooterPosition(shooterHoodSpeed);
            Shooter.setShooterPower(Shooter.PIDControl(shooterEncSpeed, Shooter.getCurrentRPM()));
            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    timer.reset();
                    stateMachine = StateMachine.SHOOT_WAIT_1;
                    break;
                case SHOOT_WAIT_1:
                    if (timer.time() > 2.0) {
                        Intake.runIntake();
                    }
                    if (timer.time() > 3.0) {
                        HelperServos.setStopperStop();
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_HUMAN_PLAYER;
                    }
                    break;
                case DRIVE_TO_HUMAN_PLAYER:
                    if (nav.driveTo(odo.getPosition(), INTAKE_HUMAN, 0.4, 1.0)) {
                        Intake.raiseIntake();
                        telemetry.addLine("about to intake at HUMAN PLAYER");
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_SHOOT_POS;
                    }
                    break;
//                case DRIVE_TO_TARGET_2:
//                    /*
//                    drive the robot to the first target, the nav.driveTo function will return true once
//                    the robot has reached the target, and has been there for (holdTime) seconds.
//                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
//                     */
//                    if (nav.driveTo(odo.getPosition(), INTAKE_HUMAN, 0.6, 0.0)) {
//                        telemetry.addLine("at position #2!");
//                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
//                    }
//                    break;
//
//                    break;
                case DRIVE_TO_SHOOT_POS:
                    if (nav.driveTo(odo.getPosition(), SHOOT_POSE, 0.35, 1.0)) {
                        telemetry.addLine("At shoot position");
                        timer.reset();
                        stateMachine = StateMachine.SHOOT_WAIT_2;
                        Intake.raiseIntake();

                    }
                    break;
                case SHOOT_WAIT_2:
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
                        stateMachine = StateMachine.BACK_TO_HUMAN_PLAYER;
                    }

                case BACK_TO_HUMAN_PLAYER:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), INTAKE_HUMAN, 0.4, 0.5)) {
                        Intake.raiseIntake();
                        HelperServos.setStopperStop();
                        telemetry.addLine("intaking at HUMAN PLAYER");
                        timer.reset();
                        stateMachine = StateMachine.DRIVE_TO_SHOOT_POS_2;
                    }
                    break;
                case DRIVE_TO_SHOOT_POS_2:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    if (nav.driveTo(odo.getPosition(), SHOOT_POSE, 0.6, 0.0)) {
                        telemetry.addLine("At shoot position");
                        stateMachine = StateMachine.DONE;
                    }
                    break;
//
//
//                case WAIT_3:
//                    if (timer.time() > 1) {
//                        HelperServos.setStopperPass();
//                    }
//                    if (timer.time() > 1.5) {
//                        Intake.lowerIntake();
//                    }
//                    if (timer.time() > 2.5) {
//                        timer.reset();
//                        Intake.raiseIntake();
//                        HelperServos.setStopperStop();
//                        stateMachine = StateMachine.DONE;
//                    }
//                    break;
                case DONE:
                    if (nav.driveTo(odo.getPosition(), odo.getPosition(), 0.7, 0.5)) {
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