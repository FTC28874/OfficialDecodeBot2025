package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.auto.Constants;

@TeleOp(name = "Pedro Turret Tracking (Constants)", group = "Control")
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
    // FIELD CONSTANTS
    // ===============================
    private static final double GOAL_X = 144.0;
    private static final double GOAL_Y = 144.0;
    private static final double GOAL_HEADING = Math.toRadians(38.0);

    // ===============================
    // TURRET OFFSET (ROBOT FRAME)
    // +X forward, +Y left
    // ===============================
    private static final double TURRET_OFFSET_X = 0.0;
    private static final double TURRET_OFFSET_Y = 0.0;

    // ===============================
    // TURRET PID CONSTANTS
    // ===============================
    private static final double kP = 3.0;
    private static final double kD = 0.15;

    private double lastError = 0.0;

    // ===============================
    // TURRET ENCODER
    // ===============================
    private static final double TICKS_PER_REV = 8192.0;

    @Override
    public void init() {

        // -------------------------------
        // Build Pedro follower USING CONSTANTS
        // -------------------------------
        follower = Constants.createFollower(hardwareMap);

        // -------------------------------
        // Set start pose (FIELD CENTER)
        // (72,72) facing +X
        // -------------------------------
        follower.setPose(new Pose(
                88,
                9,
                Math.toRadians(90)
        ));

        // -------------------------------
        // Drivetrain brake mode (optional but recommended)
        // -------------------------------
        Constants.setBrakeMode(hardwareMap);

        // -------------------------------
        // Turret motor
        // -------------------------------
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        double robotHeading = pose.getHeading();

        // -------------------------------
        // Compute turret world position
        // -------------------------------
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double turretWorldX =
                robotX + TURRET_OFFSET_X * cosH - TURRET_OFFSET_Y * sinH;
        double turretWorldY =
                robotY + TURRET_OFFSET_X * sinH + TURRET_OFFSET_Y * cosH;

        // -------------------------------
        // Vector turret → goal
        // -------------------------------
        double dx = GOAL_X - turretWorldX;
        double dy = GOAL_Y - turretWorldY;

        // -------------------------------
        // Angle math
        // -------------------------------
        double fieldAngleToGoal = Math.atan2(dy, dx);

        // Blend goal position + goal facing
        double blendedFieldAngle =
                0.85 * fieldAngleToGoal + 0.15 * GOAL_HEADING;

        double turretTargetAngle =
                wrapAngle(blendedFieldAngle - robotHeading);

        // -------------------------------
        // Turret PID
        // -------------------------------
        double currentTurretAngle = getTurretAngle();
        double error = wrapAngle(turretTargetAngle - currentTurretAngle);

        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kD * derivative;
        power = clamp(power, -1.0, 1.0);

        turretMotor.setPower(power);

        // -------------------------------
        // Telemetry
        // -------------------------------
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotHeading));
        telemetry.addData("Turret Target (deg)", Math.toDegrees(turretTargetAngle));
        telemetry.addData("Turret Error (deg)", Math.toDegrees(error));
        telemetry.update();
    }

    // ===============================
    // HELPERS
    // ===============================

    private double wrapAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double getTurretAngle() {
        double ticks = turretMotor.getCurrentPosition();
        return (ticks / TICKS_PER_REV) * (2.0 * Math.PI);
    }
}
