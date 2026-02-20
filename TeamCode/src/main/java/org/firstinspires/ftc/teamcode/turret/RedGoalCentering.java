package org.firstinspires.ftc.teamcode.turret;

/**
 * RedGoalCentering
 *
 * Concrete centering helper for AprilTag ID 24 ("RED Goal", 36h11 family).
 * Extends AprilTagCenteringBase — all logic lives in the base class.
 *
 * Usage (from a parent TeleOp):
 *
 *   RedGoalCentering redGoalCentering = new RedGoalCentering();
 *   redGoalCentering.init(hardwareMap, telemetry);   // in init block
 *   redGoalCentering.update(gamepad2.right_bumper);  // every loop tick
 *   redGoalCentering.stop();                         // on end
 */
public class RedGoalCentering extends AprilTagCenteringBase {

    private static final int    TAG_ID    = 24;
    private static final String TAG_LABEL = "RED Goal";

    public RedGoalCentering() {
        super(TAG_ID, TAG_LABEL);
    }
}
