package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.dynamicShooter;

import org.firstinspires.ftc.teamcode.robot.Shooter;

@TeleOp(name="Test XY", group="Linear OpMode")

public class TestShooter extends LinearOpMode {
    private GoBildaPinpointDriver odo = null;
    Pose2D pos = null;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        org.firstinspires.ftc.teamcode.robot.Shooter.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            odo.update();
            pos = odo.getPosition();
            double shooterEncSpeed = dynamicShooter.flywheelSpeed(dynamicShooter.distanceFromGoal(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS)));
            telemetry.addData("Robot X: ", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Robot Y: ", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Shooter Power: ", gamepad1.right_stick_y);
            telemetry.addData("Shooter rpm: ", shooterEncSpeed);
            telemetry.addData("Current Shooter RPM: ", Shooter.getCurrentRPM());
            telemetry.addData("Intake Power: ", gamepad1.left_stick_y);
            telemetry.addData("goalDist: ", dynamicShooter.distanceFromGoal(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS)));

            telemetry.update();
        }
    }
}
