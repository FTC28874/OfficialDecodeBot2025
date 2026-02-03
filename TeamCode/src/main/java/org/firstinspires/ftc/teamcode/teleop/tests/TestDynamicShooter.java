package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.DynamicShooter;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.HelperServos;
import org.firstinspires.ftc.teamcode.robot.Intake;

import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp(name = "Test Dynamic Shooter", group = "Linear OpMode")

public class TestDynamicShooter extends LinearOpMode {
    private GoBildaPinpointDriver odo = null;
    Pose2D pos = null;
    private double driverSensitivity = 1.0;
    private double shooterTargetRPM = 3000;
    private double shooterPower = 0;
    private double minHoodAngle = Shooter.HoodState.DOWN.angle; // 0.1
    private double maxHoodAngle = Shooter.HoodState.UP.angle; // 0.6

    private double curHoodAngle = minHoodAngle;

    private boolean shooterState = true;

    private double minTurretHeading = Constants.MIN_TURRET_HEAD;
    private double maxTurretHeading = Constants.MAX_TURRET_HEAD;

    private int turretPos = 0;
    private int curTurretPos = 0;
    private double goalDistance = 0;

    private double robotHeading = 0;

    private boolean isDynamic = false;
    private DcMotor driveFL = null;
    private DcMotor driveBL = null;
    private DcMotor driveFR = null;
    private DcMotor driveBR = null;
    private DcMotorEx turret = null;


    @Override
    public void runOpMode() {
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);
        HelperServos.init(hardwareMap);
        Turret.init(hardwareMap);


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
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y * driverSensitivity;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x * driverSensitivity;
            double yaw = gamepad1.right_stick_x * driverSensitivity;

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
                powerFL /= max;
                powerFR /= max;
                powerBL /= max;
                powerBR /= max;
            }

            odo.update();
            pos = odo.getPosition();
            robotHeading = pos.getHeading(AngleUnit.DEGREES);

            double curX = pos.getX(DistanceUnit.INCH);
            double curY = pos.getY(DistanceUnit.INCH);
            goalDistance = DynamicShooter.calcDistToGoal(curX, curY);
            curTurretPos = (int) Shooter.getTurretPosition();

            if (isDynamic) {
                shooterTargetRPM = DynamicShooter.calcTargetRPM(goalDistance);
                curHoodAngle = DynamicShooter.calcHoodPos(goalDistance);

                turretPos = DynamicShooter.calcTurretHead(robotHeading, curX, curY);

                int deltaTurretPos = turretPos - curTurretPos;


                turret.setTargetPosition(deltaTurretPos);
                turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                turret.setPower(0.75);
            }
            if (gamepad1.bWasPressed()) {
                isDynamic = false;
            }
            if (gamepad1.yWasPressed()) {
                isDynamic = true;
            }

            // shooter controls
            if (shooterState) {
                shooterPower = Shooter.PIDControl(shooterTargetRPM, Shooter.getCurrentRPM());
                Shooter.setShooterPower(shooterPower);
            } else {
                Shooter.stopShooter();
            }
            if (gamepad1.xWasPressed()) {
                shooterState = !shooterState;
            }

            // shooter speed controls
            if (gamepad1.dpadUpWasPressed()) {
                shooterTargetRPM = shooterTargetRPM + 100;
            }
            if (gamepad1.dpadDownWasPressed()) {
                shooterTargetRPM = shooterTargetRPM - 100;
            }

            // hood controls
            Shooter.setShooterPosition(curHoodAngle);

            if (gamepad1.dpadRightWasPressed()) {
                if (curHoodAngle < maxHoodAngle) {
                    curHoodAngle += 0.05;
                }
            }
            if (gamepad1.dpadLeftWasPressed()) {
                if (curHoodAngle > minHoodAngle) {
                    curHoodAngle -= 0.05;
                }
            }

            // turret heading controls
            if (gamepad1.left_trigger > 0.1) {
                if (Shooter.getTurretPosition() > minTurretHeading) {
                    Shooter.turnTurretDirection(false, 0.4);
                }
            }
            if (gamepad1.right_trigger > 0.1) {
                if (Shooter.getTurretPosition() < maxTurretHeading) {
                    Shooter.turnTurretDirection(true, 0.4);
                }
            }
            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
                Shooter.turnTurretDirection(true, 0);
            }

            // intake controls
            if (gamepad1.left_bumper) {
                Intake.runIntake();
                HelperServos.setStopperStop();
                Intake.raiseIntake();
            } else {
                Intake.stopIntake();
            }
            if (gamepad1.right_bumper) {
                Intake.lowerIntake();
                Intake.runIntake();
                HelperServos.setStopperPass();
            }

            // reset position to 0,0 and hood to 0.1
            if (gamepad1.aWasPressed()) {
                odo.resetPosAndIMU();
                curHoodAngle = minHoodAngle;
            }
            driveFL.setPower(powerFL);
            driveFR.setPower(powerFR);
            driveBL.setPower(powerBL);
            driveBR.setPower(powerBR);

            telemetry.addData("Dynamic Shooting Active: ", isDynamic);
            telemetry.addData("Robot X: ", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y: ", pos.getY(DistanceUnit.INCH));
            telemetry.addData("goalDist: ", goalDistance);
            telemetry.addData("Robot Heading: ", robotHeading);
            telemetry.addData("Shooter Target RPM: ", shooterTargetRPM);
            telemetry.addData("Hood Angle", curHoodAngle);
            telemetry.addData("Current Turret Heading: ", turret.getCurrentPosition());
            telemetry.addData("Target Turret Position: ", turretPos);
            telemetry.addData("Shooter ON: ", shooterState);
            telemetry.addData("Current Shooter RPM: ", Shooter.getCurrentRPM());
            telemetry.update();
        }
    }
}
