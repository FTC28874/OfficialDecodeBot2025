package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * BlueGoalCenteringTeleOp — DECODE Season (2025-2026)
 *
 * Tracks AprilTag ID 20 (BLUE Goal, 36h11 family, 8" outer / 6.5" inner)
 * and keeps it centered in the camera frame by panning the camera motor.
 *
 * All logic lives in {@link AprilTagCenteringBase}.
 * All tuning constants live in {@link PanMotorConfig}.
 */
@TeleOp(name = "BLUE Goal Centering", group = "Vision")
public class BlueGoalCenteringTeleOp extends AprilTagCenteringBase {

    @Override
    protected int getTargetTagId() {
        return 20;
    }

    @Override
    protected String getTargetTagName() {
        return "BLUE Goal";
    }
}
