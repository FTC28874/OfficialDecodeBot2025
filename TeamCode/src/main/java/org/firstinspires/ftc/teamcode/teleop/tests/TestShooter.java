package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Shooter;

@TeleOp(name="Test Shooter 123", group="Linear OpMode")
public class TestShooter extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor shooterU = null;
    private DcMotor shooterD = null;
    private DcMotor intake = null;
    private double shooterEncSpeed = 2300;


    @Override
    public void runOpMode() {

        shooterU = hardwareMap.get(DcMotor.class, "shooterU");
        shooterD = hardwareMap.get(DcMotor.class, "shooterD");
        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterU.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterD.setDirection(DcMotorSimple.Direction.FORWARD);

        org.firstinspires.ftc.teamcode.robot.Shooter.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

//            shooterU.setPower(gamepad1.right_stick_y);
//            shooterD.setPower(gamepad1.right_stick_y);
            if (gamepad1.a) {
//                shooterU.setPower(0.7);
//                shooterD.setPower(0.7);
                Shooter.SetShooterPower(Shooter.PIDControl(shooterEncSpeed, Shooter.GetCurrentRPM()));
            } else if (!gamepad1.a) {
//                shooterU.setPower(0);
//                shooterD.setPower(0);
                Shooter.StopShooter();
            }

            if (gamepad1.dpad_up) {
                shooterEncSpeed = shooterEncSpeed + 50;
                while (gamepad1.dpad_up) {}
            }
            if (gamepad1.dpad_down) {
                shooterEncSpeed = shooterEncSpeed - 50;
                while (gamepad1.dpad_down) {}
            }

            intake.setPower(gamepad1.left_stick_y);

            telemetry.addData("Shooter Power: ", gamepad1.right_stick_y);
            telemetry.addData("Shooter rpm: ", shooterEncSpeed);
            telemetry.addData("Current Shooter RPM: ", Shooter.GetCurrentRPM());
            telemetry.addData("Intake Power: ", gamepad1.left_stick_y);

            telemetry.update();
        }
    }
}
