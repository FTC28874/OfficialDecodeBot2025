package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Shooter;

@TeleOp(name="Test Shooter 123", group="Linear OpMode")
public class TestShooter extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor shooter = null;
    private DcMotor intake = null;


    @Override
    public void runOpMode() {

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();
        while (opModeIsActive()) {

            shooter.setPower(gamepad1.right_stick_y);
            intake.setPower(gamepad1.left_stick_y);

            telemetry.addData("Shooter Power: ", gamepad1.right_stick_y);
            telemetry.addData("Intake Power: ", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
