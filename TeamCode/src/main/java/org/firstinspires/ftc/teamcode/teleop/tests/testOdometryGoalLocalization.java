package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.Constants;

@TeleOp(name = "Pedro Turret Tracking Degrees", group = "Control")
public class testOdometryGoalLocalization extends OpMode {

    // ===============================
    // PEDRO FOLLOWER
    // ===============================
    private Follower follower;

    // ===============================
    // TURRET HARDWARE
    // ===============================
    private DcMotorEx turretMotor;

    // ===============================
    // FIELD GOAL
    // ===============================
    private static final double GOAL_X = 144.0;
    private static final double GOAL_Y = 144.0;
    private static final double GOAL_HEADING_DEG = 38.0;

    // ===============================
    // TURRET OFFSET
    // ===============================
    private static final double TURRET_OFFSET_X = 0.0;
    private static final double TURRET_OFFSET_Y = 0.0;

    // ===============================
    // TURRET PID (degrees)
    // ===============================
    private static final double kP_DEG = 0.01; // small for degrees
    private double lastErrorDeg = 0.0;

    // ===============================
    // TICKS PER REV (GoBILDA 435 RPM)
    // ===============================
    private static final double TICKS_PER_REV = 384;

    // ===============================
    // HARD LIMITS (degrees)
    // ===============================
    private static final double TURRET_MIN_DEG = -135.0;
    private static final double TURRET_MAX_DEG = 135.0;

    @Override
    public void init() {

        // -------------------------------
        // Build Pedro follower USING CONSTANTS
        // -------------------------------
        follower = Constants.createFollower(hardwareMap);

        // -------------------------------
        // Set start pose (field-specific)
        // -------------------------------
        follower.setPose(new Pose(
                88,
                9,
                Math.toRadians(90)
        ));

        // -------------------------------
        // Set drivetrain brake mode
        // -------------------------------
        Constants.setBrakeMode(hardwareMap);

        // -------------------------------
        // Initialize turret motor
        // -------------------------------
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        // -------------------------------
        // Update Pedro localization
        // -------------------------------
        follower.update();
        Pose pose = follower.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingRad = pose.getHeading();
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        // -------------------------------
        // Compute turret world position
        // -------------------------------
        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);
        double turretWorldX = robotX + TURRET_OFFSET_X * cosH - TURRET_OFFSET_Y * sinH;
        double turretWorldY = robotY + TURRET_OFFSET_X * sinH + TURRET_OFFSET_Y * cosH;

        // -------------------------------
        // Compute target turret angle (degrees)
        // -------------------------------
        double targetTurretDeg = GOAL_HEADING_DEG - robotHeadingDeg;

        // Apply hard limits
        targetTurretDeg = clamp(targetTurretDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);

        // -------------------------------
        // Read current turret angle (degrees)
        // -------------------------------
        double currentTurretDeg = (turretMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;

        // -------------------------------
        // PID error
        // -------------------------------
        double errorDeg = targetTurretDeg - currentTurretDeg;
        errorDeg = wrapErrorDeg(errorDeg);

        // Simple P controller
        double power = kP_DEG * errorDeg;
        power = clamp(power, -0.5, 0.5);

        // Safety: stop motor if hitting limits
        if ((currentTurretDeg <= TURRET_MIN_DEG && power < 0) ||
                (currentTurretDeg >= TURRET_MAX_DEG && power > 0)) {
            power = 0;
        }

        turretMotor.setPower(power);

        // -------------------------------
        // Telemetry
        // -------------------------------
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading (deg)", robotHeadingDeg);
        telemetry.addData("Turret Current (deg)", currentTurretDeg);
        telemetry.addData("Turret Target (deg)", targetTurretDeg);
        telemetry.addData("Error (deg)", errorDeg);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }

    // ===============================
    // HELPERS
    // ===============================

    private double wrapErrorDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
