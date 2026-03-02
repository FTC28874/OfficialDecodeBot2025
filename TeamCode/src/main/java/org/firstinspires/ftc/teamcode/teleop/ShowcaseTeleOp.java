package org.firstinspires.ftc.teamcode.teleop;


import androidx.appcompat.widget.ThemedSpinnerAdapter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.HelperServos;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "Showcase TeleOp", group = "Linear OpMode")
public class ShowcaseTeleOp extends LinearOpMode {


    private boolean stopperServo = false;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        Intake.init(hardwareMap);
        Shooter.init(hardwareMap);
        HelperServos.init(hardwareMap);

        Intake.raiseIntake();
        Shooter.lowerShooter();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            // Shooter Control
            if (gamepad1.right_bumper) {
                Shooter.setShooterPower(Shooter.PIDControl(500, Shooter.getCurrentRPM()));
            } else {
                Shooter.stopShooter();
            }

            // Intake control
            if (gamepad1.left_bumper) {
                Intake.runIntakeSlow();
            } else {
                Intake.stopIntake();
            }

            // Stopper Servo Control
            if (gamepad1.bWasPressed()) {
                stopperServo = !stopperServo;
            }
            if (stopperServo) {
                HelperServos.setStopperStop();
            } else {
                HelperServos.setStopperPass();
            }

            // Other Servos Control
            if (gamepad1.dpadUpWasPressed()) {
                Shooter.raiseShooter();
            }
            if (gamepad1.dpadDownWasPressed()) {
                Shooter.lowerShooter();
            }
            if (gamepad1.dpadLeftWasPressed()) {
                Intake.lowerIntake();
            }
            if (gamepad1.dpadRightWasPressed()) {
                Intake.raiseIntake();
            }
        }
    }
}
