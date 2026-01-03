package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.aprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "AprilTag X Align Test", group = "Linear OpMode")
public class testAprilTag extends LinearOpMode {

    private aprilTagWebcam aprilTagWebcam = null;
    private DcMotor alignMotor= null;

    // Tunables
    private static final int TARGET_TAG_ID = 21;
    private static final double MOTOR_SPEED = 0.4;
    private static final double DEADZONE_CM = 10; // stop when |x| <= this

    @Override
    public void runOpMode() {
        aprilTagWebcam = new aprilTagWebcam();
        aprilTagWebcam.init(hardwareMap, telemetry);

        alignMotor = hardwareMap.get(DcMotor.class, "turret");
        alignMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        alignMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Flip this if the motor spins the wrong way
        alignMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized. Start OpMode.");

        waitForStart();
        while (opModeIsActive()) {
            aprilTagWebcam.update();

            AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

            if (tag != null) {
                double x = tag.ftcPose.x;

                if (Math.abs(x) <= DEADZONE_CM) {
                    // Centered
                    alignMotor.setPower(0);
                    telemetry.addLine("Aligned: motor stopped");
                }
                else if (x > 0) {
                    // Tag is to the right → move left
                    alignMotor.setPower(-MOTOR_SPEED);
                    telemetry.addLine("Tag right → motor left");
                }
                else {
                    // Tag is to the left → move right
                    alignMotor.setPower(MOTOR_SPEED);
                    telemetry.addLine("Tag left → motor right");
                }

                telemetry.addData("Tag X (cm)", x);
                aprilTagWebcam.displayDetectionTelemetry(tag);
            }
            else {
                // No tag detected → stop motor for safety
                alignMotor.setPower(0);
                telemetry.addLine("Tag not detected");
            }

            telemetry.update();
        }

        alignMotor.setPower(0);
        aprilTagWebcam.stop();
    }
}
