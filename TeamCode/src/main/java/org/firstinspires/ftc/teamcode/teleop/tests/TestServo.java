package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Intake;

@TeleOp(name="Test Servo 123", group = "Linear OpMode")
public class TestServo extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake = null;
    private Servo intakeServo = null;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        org.firstinspires.ftc.teamcode.robot.Intake.init(hardwareMap);

        Intake.raiseIntake();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpadDownWasPressed()) {
                Intake.lowerIntake();
            }
            if (gamepad1.dpadUpWasPressed()) {
                Intake.raiseIntake();
            }
            if (gamepad1.aWasPressed()) {
                Intake.setIntakeToPosition(0.6);
            }

            if (gamepad1.left_bumper) {
                Intake.runIntake();
            }
            if (!gamepad1.left_bumper) {
                Intake.stopIntake();
            }
        }
    }

}
