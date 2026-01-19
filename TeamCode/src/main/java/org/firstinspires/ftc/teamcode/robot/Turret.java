package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret helper
 * Uses Pinpoint odometry X/Y + heading
 * Encoder read from driveFR
 */
public final class Turret {

    /* ================= HARDWARE ================= */

    private static DcMotorEx turretMotor;
    private static DcMotorEx encoderMotor; // driveFR encoder only

    /* ================= CONSTANTS ================= */

    private static final double MOTOR_TICKS_PER_REV = 537.6; // 435 RPM YJ
    private static final double GEAR_RATIO = 125.0 / 24.0;

    private static final double MIN_ANGLE_DEG = -90.0;
    private static final double MAX_ANGLE_DEG =  90.0;

    private static final double kP = 0.008;
    private static final double MAX_POWER = 0.45;
    private static final double ANGLE_DEADBAND_DEG = 1.0;

    /* ================= FIELD ================= */

    // Decode field, inches, origin at center
    private static final double RED_GOAL_X =  68.0;
    private static final double RED_GOAL_Y = -68.0;

    /* ================= STATE ================= */

    private static boolean initialized = false;
    private static double zeroOffsetTicks = 0.0;

    /* ================= INIT ================= */

    public static void init(HardwareMap hardwareMap) {
        turretMotor  = hardwareMap.get(DcMotorEx.class, "turret");
        encoderMotor = hardwareMap.get(DcMotorEx.class, "driveFR");

        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        initialized = true;
    }

    /** Call ONCE when turret is physically centered */
    public static void zeroTurret() {
        if (!initialized) return;
        zeroOffsetTicks = encoderMotor.getCurrentPosition();
    }

    /* ================= MAIN LOOP ================= */

    /**
     * Aim turret at red goal using FULL field-centric math
     */
    public static void aimAtRedGoal(
            double robotX,
            double robotY,
            double robotHeadingDeg
    ) {
        if (!initialized) return;

        // 1️⃣ Field angle from robot to goal
        double fieldAngleDeg = calculateFieldAngleToGoal(robotX, robotY);

        // 2️⃣ Subtract robot heading (THIS IS THE KEY STEP)
        double turretTargetDeg =
                normalizeAngle(fieldAngleDeg - robotHeadingDeg);

        // 3️⃣ Enforce hard limits
        turretTargetDeg = Range.clip(
                turretTargetDeg,
                MIN_ANGLE_DEG,
                MAX_ANGLE_DEG
        );

        // 4️⃣ Drive turret
        setTurretAngle(turretTargetDeg);
    }

    /* ================= CONTROL ================= */

    public static void setTurretAngle(double targetDeg) {
        double currentDeg = getCurrentTurretAngleDeg();
        double errorDeg = targetDeg - currentDeg;

        if (Math.abs(errorDeg) < ANGLE_DEADBAND_DEG) {
            turretMotor.setPower(0);
            return;
        }

        double errorTicks =
                degreesToTicks(targetDeg) - getCurrentTurretTicks();

        double power = Range.clip(
                errorTicks * kP,
                -MAX_POWER,
                MAX_POWER
        );

        turretMotor.setPower(power);
    }

    public static void stop() {
        if (!initialized) return;
        turretMotor.setPower(0);
    }

    /* ================= INTERNAL ================= */

    /** FTC-aligned atan2: 0° = forward, +90° = left */
    private static double calculateFieldAngleToGoal(double robotX, double robotY) {
        double dx = RED_GOAL_X - robotX;
        double dy = RED_GOAL_Y - robotY;
        return Math.toDegrees(Math.atan2(dx, dy));
    }

    public static double getCurrentTurretTicks() {
        return -(encoderMotor.getCurrentPosition() - zeroOffsetTicks);
    }

    private static double getCurrentTurretAngleDeg() {
        return ticksToDegrees(getCurrentTurretTicks());
    }

    private static double degreesToTicks(double deg) {
        double turretRevs = deg / 360.0;
        double motorRevs  = turretRevs * GEAR_RATIO;
        return motorRevs * MOTOR_TICKS_PER_REV;
    }

    private static double ticksToDegrees(double ticks) {
        double motorRevs  = ticks / MOTOR_TICKS_PER_REV;
        double turretRevs = motorRevs / GEAR_RATIO;
        return turretRevs * 360.0;
    }

    private static double normalizeAngle(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
