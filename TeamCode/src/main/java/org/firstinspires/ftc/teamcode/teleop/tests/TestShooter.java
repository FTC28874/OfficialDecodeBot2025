package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.PIDtoPoint.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.robot.dynamicShooter;

import org.firstinspires.ftc.teamcode.robot.Shooter;

@TeleOp(name="Test Shooter 123", group="Linear OpMode")
@Disabled
public class TestShooter extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor shooterU = null;
    private DcMotor shooterD = null;
    private DcMotor intake = null;
    private GoBildaPinpointDriver odo = null;
    Pose2D pos = null; //odo.getPosition(); ✅ ️




    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-48, -156, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        odo.resetPosAndIMU();

        org.firstinspires.ftc.teamcode.robot.Shooter.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

//            shooterU.setPower(gamepad1.right_stick_y);
//            shooterD.setPower(gamepad1.right_stick_y);
            odo.update();
            pos = odo.getPosition();
            double shooterEncSpeed = dynamicShooter.flywheelSpeed(dynamicShooter.distanceFromGoal(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS)));
//            Shooter.setShooterPower();

            telemetry.addData("Shooter Power: ", gamepad1.right_stick_y);
            telemetry.addData("Shooter rpm: ", shooterEncSpeed);
            telemetry.addData("Current Shooter RPM: ", Shooter.getCurrentRPM());
            telemetry.addData("Intake Power: ", gamepad1.left_stick_y);
            telemetry.addData("goalDist: ", dynamicShooter.distanceFromGoal(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS)));

            telemetry.update();
        }
    }
}
