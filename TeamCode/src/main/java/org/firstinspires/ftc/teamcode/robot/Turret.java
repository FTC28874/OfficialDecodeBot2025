package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret helper
 * Reads encoder from driveFL (shared with drivetrain)
 */
public final class Turret {

    /* ------------------ HARDWARE ----------------- */
    private static DcMotorEx turretMotor;
    private static DcMotorEx encoderMotor; // driveFL

    /* ------------------ CONSTANTS ------------------ */

    private static final double MOTOR_TICKS_PER_REV = 537.6;
    private static final double GEAR_RATIO = 125.0 / 24.0;

    private static final double MAX_ANGLE_DEG = 90.0;
    private static final double MIN_ANGLE_DEG = -90.0;

    private static final double kP = 0.01;

    /* ------------------ FIELD ------------------ */
    private static final double RED_GOAL_X = 72.0;
    private static final double RED_GOAL_Y = 144.0;

    /* ------------------ STATE ------------------ */
    private static boolean initialized = false;
    private static double turretZeroOffsetTicks = 0;

    /* ------------------ INIT ------------------ */

    public static void init(HardwareMap hardwareMap) {

        turretMotor  = hardwareMap.get(DcMotorEx.class, "turret");
        encoderMotor = hardwareMap.get(DcMotorEx.class, "driveFR");

        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // DO NOT touch encoderMotor power or mode
        initialized = true;
    }

    /**
     * Call once when turret is physically centered
     */
    public static void zeroTurret() {
        double turretPosition = -(encoderMotor.getCurrentPosition());
        if (!initialized) return;
        turretZeroOffsetTicks = encoderMotor.getCurrentPosition();
    }

    /* ------------------ LOOP ------------------ */

    public static void aimAtRedGoal(double robotX, double robotY, double robotHeadingDeg) {
        if (!initialized) return;

        double fieldTargetAngleDeg = calculateFieldTargetAngle(robotX, robotY);
        double turretTargetAngleDeg =
                normalizeAngle(fieldTargetAngleDeg - robotHeadingDeg);
        double turretPosition = -(encoderMotor.getCurrentPosition());

        turretTargetAngleDeg = Range.clip(
                turretTargetAngleDeg,
                MIN_ANGLE_DEG,
                MAX_ANGLE_DEG
        );

        setTurretAngle(turretTargetAngleDeg);
    }

    public static void setTurretAngle(double targetAngleDeg) {
        if (!initialized) return;

        targetAngleDeg = Range.clip(
                targetAngleDeg,
                MIN_ANGLE_DEG,
                MAX_ANGLE_DEG
        );

        double targetTicks = degreesToTicks(targetAngleDeg);
        double error = targetTicks - getCurrentTurretTicks();

        double power = Range.clip(error * kP, -1.0, 1.0);
        turretMotor.setPower(power);
    }

    public static void stop() {
        if (!initialized) return;
        turretMotor.setPower(0);
    }

    /* ------------------ INTERNAL ------------------ -*/

    private static double getCurrentTurretTicks() {
        double turretPosition = -(encoderMotor.getCurrentPosition());
        return encoderMotor.getCurrentPosition() - turretZeroOffsetTicks;
    }

    private static double calculateFieldTargetAngle(double robotX, double robotY) {
        double dx = RED_GOAL_X - robotX;
        double dy = RED_GOAL_Y - robotY;
        return Math.toDegrees(Math.atan2(dx, dy));
    }

    private static double degreesToTicks(double degrees) {
        double turretRevs = degrees / 360.0;
        double motorRevs = turretRevs * GEAR_RATIO;
        return motorRevs * MOTOR_TICKS_PER_REV;
    }

    private static double normalizeAngle(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }
}
