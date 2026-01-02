package org.firstinspires.ftc.teamcode.teleop.tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.aprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@TeleOp

public class testAprilTag extends OpMode {
    aprilTagWebcam aprilTagWebcam = new aprilTagWebcam();
    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);

        aprilTagWebcam.displayDetectionTelemetry(id24);

        if (id24 != null) {
            telemetry.addData("id24 string", id24.toString());
        } else {
            telemetry.addData("id24 string", "Tag 24 not detected");
        }
    }

}//asdfa