package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.HelperServos;
import org.firstinspires.ftc.teamcode.robot.Intake;

import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp(name="Test XY", group="Linear OpMode")

public class TestShooter extends LinearOpMode {
    private GoBildaPinpointDriver odo = null;
    Pose2D pos = null;

    private double shooterTargetRPM = 3000;
    private double shooterPower = 0;
    private double minHoodAngle = Shooter.HoodState.DOWN.angle; // 0.1
    private double maxHoodAngle = Shooter.HoodState.UP.angle; // 0.6

    private double curHoodAngle = minHoodAngle;

    private boolean shooterState = true;

    private double minTurretHeading = -500;
    private double maxTurretHeading = 450;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);
        HelperServos.init(hardwareMap);
        Turret.init(hardwareMap);

        telemetry.addLine("Stopper Servo Init");

        waitForStart();
        while (opModeIsActive()) {
            odo.update();
            pos = odo.getPosition();

            // shooter controls
            if (shooterState) {
                shooterPower = Shooter.PIDControl(shooterTargetRPM, Shooter.getCurrentRPM());
                Shooter.setShooterPower(shooterPower);
            }
            else {
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
            }
            else {
                Intake.stopIntake();
            }

            telemetry.addData("Robot X: ", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y: ", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Shooter Target RPM: ", shooterTargetRPM);
            telemetry.addData("Current Shooter RPM: ", Shooter.getCurrentRPM());
            telemetry.addData("Hood Angle", curHoodAngle);
            telemetry.addData("Shooter ON: ", shooterState);
            telemetry.addData("Turret Heading: ", Shooter.getTurretPosition());

            telemetry.update();
        }
    }
}
