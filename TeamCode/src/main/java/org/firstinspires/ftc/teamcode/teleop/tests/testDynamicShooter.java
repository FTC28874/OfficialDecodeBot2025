package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.HelperServos;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name = "Test Dynamic Shooter 123", group = "Linear OpMode")
public class testDynamicShooter extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint = null;
    double shooterEncSpeed = 1800;
    double shooterHoodAngle = Shooter.ShootPositionState.CLOSE_HOOD.position;


    public void runOpMode() {
        Turret.init(hardwareMap);
        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);
        HelperServos.init(hardwareMap);
        pinpoint.setOffsets(-48, -156); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver.GoBildapinpointmetryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver.EncoderDirection.FORWARD, org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();
        waitForStart();
        while (opModeIsActive()) {
            pinpoint.update();

//            telemetry.addData("Distance to goal: ", Turret.getDistanceToGoal(pinpoint.getPosition()));
            telemetry.addData("Robot X ", pinpoint.getXOffset(DistanceUnit.INCH));
            telemetry.addData("Robot Y ", pinpoint.getYOffset(DistanceUnit.INCH));
            telemetry.addData("Shooter Target RPM: ", shooterEncSpeed);
            telemetry.addData("Shooter RPM: ", Shooter.getCurrentRPM());
            telemetry.addData("Shooter Hood Angle: ", shooterHoodAngle);
            telemetry.addData("Turret Position", Turret.getCurrentAngle());
            telemetry.update();

            if (gamepad1.dpadUpWasPressed()) {
                shooterEncSpeed = shooterEncSpeed + 50;
            }
            if (gamepad1.dpadDownWasPressed()) {
                shooterEncSpeed = shooterEncSpeed - 50;
            }

            Shooter.setShooterPosition(shooterHoodAngle);

            if (gamepad1.dpadUpWasPressed()) {
                shooterHoodAngle = Shooter.HoodState.UP.angle;
            }
            if (gamepad1.dpadDownWasPressed()) {
                shooterHoodAngle = Shooter.HoodState.DOWN.angle;
            }
            if (gamepad1.dpadRightWasPressed()) {
                if (shooterHoodAngle < Shooter.HoodState.UP.angle && shooterHoodAngle >= Shooter.HoodState.DOWN.angle) {
                    shooterHoodAngle = shooterHoodAngle + 0.05;
                }
            }
            if (gamepad1.dpadLeftWasPressed()) {
                if (shooterHoodAngle <= Shooter.HoodState.UP.angle && shooterHoodAngle > Shooter.HoodState.DOWN.angle) {
                    shooterHoodAngle = shooterHoodAngle - 0.05;
                }
            }

            if (gamepad1.right_bumper) {
                Shooter.setShooterPower(Shooter.PIDControl(shooterEncSpeed, Shooter.getCurrentRPM()));
            }
            if (!gamepad1.right_bumper) {
                Shooter.setShooterPower(0.0);
            }
            if (gamepad1.left_bumper) {
                Intake.runIntake();
                Intake.raiseIntake();
            }
            if (gamepad1.bWasPressed()) {
                HelperServos.setStopperStop();
            }
            if (gamepad1.aWasPressed()) {
                HelperServos.setStopperPass();
            }
            if (!gamepad1.left_bumper) {
                Intake.stopIntake();
            }
            if (gamepad1.left_trigger > 0.1) {
                Shooter.turnTurretDirection(false, 0.4);
            }
            if (gamepad1.right_trigger > 0.1) {
                Shooter.turnTurretDirection(true, 0.4);
            }
            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
                Shooter.turnTurretDirection(true, 0);



            }

        }
    }
}
