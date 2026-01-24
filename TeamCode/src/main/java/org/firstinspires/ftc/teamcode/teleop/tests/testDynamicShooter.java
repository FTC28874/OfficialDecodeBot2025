package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.HelperServos;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;

@TeleOp(name = "Test Dynamic Shooter 123", group = "Linear OpMode")
public class testDynamicShooter extends LinearOpMode {

    private GoBildaPinpointDriver odo = null;
    double shooterEncSpeed = 1800;
    double shooterHoodAngle = Shooter.ShootPositionState.CLOSE_HOOD.position;


    public void runOpMode() {
        Turret.init(hardwareMap);
        Shooter.init(hardwareMap);
        Intake.init(hardwareMap);
        HelperServos.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance to goal: ", Turret.getDistanceToGoal(odo.getPosition()));
            telemetry.addData("Robot Position: ", odo.getPosition());
            telemetry.addData("Shooter Target RPM: ", shooterEncSpeed);
            telemetry.addData("Shooter RPM: ", Shooter.getCurrentRPM());
            telemetry.addData("Shooter Hood Angle: ", shooterHoodAngle);
            telemetry.update();

            if (gamepad1.dpadUpWasPressed()) {
                shooterEncSpeed = shooterEncSpeed + 50;
            }
            if (gamepad1.dpadDownWasPressed()) {
                shooterEncSpeed = shooterEncSpeed - 50;
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

        }
    }
}
