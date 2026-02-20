package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * RedGoalCentering
 *
 * TeleOp OpMode that keeps AprilTag ID 24 ("RED Goal", 36h11 family)
 * horizontally centred in the camera frame by driving the turret motor.
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  This file intentionally contains NO logic.                     │
 * │  All behaviour lives in AprilTagCenteringBase.                  │
 * │  All tuning parameters live in TurretMotorConfig.               │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * Tag spec
 * --------
 *   ID      : 24
 *   Family  : 36h11
 *   Outer   : 8 in  (≈ 203.2 mm)
 *   Dark    : 6.5 in (≈ 165.1 mm)
 *
 * Hardware
 * --------
 *   Motor   : GoBilda 5203-2402-0014  → configured as "turret"
 *   Camera  : Logitech C270           → configured as "Webcam 1"
 *   Limits  : ±45 ° from centre  (≈ ±48 encoder ticks)
 */
@TeleOp(name = "Red Goal Centering", group = "Turret")
public class RedGoalCentering extends AprilTagCenteringBase {

    /** AprilTag ID to track. */
    private static final int    TAG_ID    = 24;

    /** Human-readable label shown on the Driver Station. */
    private static final String TAG_LABEL = "RED Goal";

    public RedGoalCentering() {
        super(TAG_ID, TAG_LABEL);
    }
}
