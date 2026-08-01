package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test Motor Video", group = "Tests")
public class testMotorVideo extends LinearOpMode {

    // Declare the motor variable
    private DcMotor testMotor;

    @Override
    public void runOpMode() {
        // Map the motor to the hardware configuration on the Driver Station
        // Ensure Port 0 on the Control Hub is named "motor0" (or update the string below)
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        // Optional: Set motor direction if it spins backward from intended movement
        testMotor.setDirection(DcMotor.Direction.FORWARD);

        // Optional: Set behavior when power is 0 (BRAKE prevents coasting)
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry feedback that setup is complete
        telemetry.addData("Status", "Initialized. Press Play to start.");
        telemetry.update();

        // Wait for the driver to press PLAY on the Driver Station
        waitForStart();

        // Loop continuously while the OpMode is active
        while (opModeIsActive()) {

            // Read Y-axis stick input (Invert gamepad Y axis because pushing UP gives a negative value in FTC)
            double motorPower = -gamepad1.left_stick_y;

            // Apply power to the motor plugged into Port 0
            testMotor.setPower(motorPower);

            // Display motor telemetry on Driver Station
            telemetry.addData("Motor Power Applied", motorPower);
            telemetry.update();
        }
    }
}