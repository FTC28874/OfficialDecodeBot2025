package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.HelperServos;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;

@TeleOp(name = "Kid's TeleOp", group = "Linear OpMode")
public class KidsTeleOp extends LinearOpMode {
    private double driverSensitivity = 0.3;
    private double shooterTargetRPM = 700.0;
    private double shooterPower = 0;
    private boolean shooterState = false;
    private double curHoodAngle = Shooter.HoodState.DOWN.angle;
    private boolean override = false;


    private DcMotor driveFL = null;
    private DcMotor driveBL = null;
    private DcMotor driveFR = null;
    private DcMotor driveBR = null;
    private DcMotor turret = null;

    @Override
    public void runOpMode() {
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter.init(hardwareMap);
        HelperServos.init(hardwareMap);
        Intake.init(hardwareMap);

        Intake.raiseIntake();
        Shooter.lowerShooter();

        waitForStart();
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y * driverSensitivity;
            double lateral =  gamepad1.left_stick_x * driverSensitivity;
            double yaw     =  gamepad1.right_stick_x * driverSensitivity;

            double powerFL = axial + lateral + yaw;
            double powerFR = axial - lateral - yaw;
            double powerBL = axial - lateral + yaw;
            double powerBR = axial + lateral - yaw;

            max = Math.max(Math.abs(powerFL), Math.abs(powerFR));
            max = Math.max(max, Math.abs(powerBL));
            max = Math.max(max, Math.abs(powerBR));

            if (max > 1.0) {
                powerFL /= max;
                powerFR /= max;
                powerBL /= max;
                powerBR /= max;
            }

            if (gamepad1.x) {
                Intake.runIntake();
                HelperServos.setStopperPass();
            }
            if (gamepad1.left_bumper) {
                Intake.runIntake();
                HelperServos.setStopperStop();
            }


            Shooter.setShooterPosition(curHoodAngle);
            if (gamepad1.dpadLeftWasPressed()) {
                if (curHoodAngle > Shooter.HoodState.DOWN.angle) {
                    curHoodAngle -= 0.05;
                }
            }
            if (gamepad1.dpadRightWasPressed()) {
                if (curHoodAngle < Shooter.HoodState.UP.angle) {
                    curHoodAngle += 0.05;
                }
            }
            if (gamepad2.rightBumperWasPressed()) {
                shooterState = !shooterState;
            }
            if (gamepad2.dpadDownWasPressed()) {
                shooterTargetRPM -= 50;
            }
            if (gamepad2.dpadUpWasPressed()) {
                shooterTargetRPM += 50;
            }
            shooterPower = Shooter.PIDControl(shooterTargetRPM, Shooter.getCurrentRPM());
            Shooter.setShooterPower(shooterPower);
            if (shooterState) {
                shooterTargetRPM = 500;
            } else {
                shooterTargetRPM = 0;
            }

            if (override) {
                driverSensitivity = 0.0;
                shooterTargetRPM = 0;
                Intake.stopIntake();
                HelperServos.setStopperStop();

            } else {
                driverSensitivity = 0.3;
            }
            if (gamepad2.xWasPressed()) {
                override = !override;
            }

            telemetry.addData("Driver Sensitivity", driverSensitivity);
            telemetry.addData("Shooter RPM", shooterTargetRPM);
            telemetry.addData("OVERRIDE", override);
            telemetry.addLine("+++++++");
            telemetry.addData("FL", powerFL);
            telemetry.addData("FR", powerFR);
            telemetry.addData("BL", powerBL);
            telemetry.addData("BR", powerBR);
        }
    }
}


