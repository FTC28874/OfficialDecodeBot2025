package org.firstinspires.ftc.teamcode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.Constants;

@TeleOp(name = "Pedro Turret Tracking (LIMITED + DEG)", group = "Control")
public class TestShooter extends OpMode {

    // ===============================
    // PEDRO
    // ===============================
    private Follower follower;

    // ===============================
    // TURRET MOTOR
    // ===============================
    private DcMotorEx turretMotor;

    // ===============================
    // GOAL (FIELD)
    // ===============================
    private static final double GOAL_HEADING_DEG = 38.0;

    // ===============================
    // TURRET MOTOR CONSTANTS
    // ===============================
    private static final double TICKS_PER_REV = 384.0; // GoBilda 435 RPM
    private static final double GEAR_RATIO = 1.0;      // change if geared

    // ===============================
    // TURRET LIMITS (DEGREES)
    // ===============================
    private static final double MIN_TURRET_DEG = -90.0;
    private static final double MAX_TURRET_DEG = 90.0;

    // ===============================
    // PID
    // ===============================
    private static final double kP = 0.02;
    private static final double kD = 0.001;

    private double lastErrorDeg = 0.0;

    @Override
    public void init() {

        // Pedro follower
        follower = Constants.createFollower(hardwareMap);

        // Set start pose (FIELD CENTER if you want)
        follower.setPose(new Pose(
                72,
                72,
                Math.toRadians(0)
        ));

        Constants.setBrakeMode(hardwareMap);

        // Turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT: encoder zero = turret parallel
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        // Update Pedro
        follower.update();
        Pose pose = follower.getPose();

        // Robot heading (deg)
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        // ===============================
        // TARGET TURRET ANGLE
        // ===============================
        double targetTurretDeg =
                wrapDeg(GOAL_HEADING_DEG - robotHeadingDeg);

        // Apply turret limits
        targetTurretDeg = clamp(
                targetTurretDeg,
                MIN_TURRET_DEG,
                MAX_TURRET_DEG
        );

        // ===============================
        // CURRENT TURRET ANGLE
        // ===============================
        double currentTurretDeg = getTurretAngleDeg();

        // ===============================
        // PD CONTROL
        // ===============================
        double errorDeg = wrapDeg(targetTurretDeg - currentTurretDeg);
        double derivative = errorDeg - lastErrorDeg;
        lastErrorDeg = errorDeg;

        double power = kP * errorDeg + kD * derivative;
        power = clamp(power, -0.6, 0.6);

        // ===============================
        // SAFETY: STOP AT LIMITS
        // ===============================
        if ((currentTurretDeg <= MIN_TURRET_DEG && power < 0) ||
                (currentTurretDeg >= MAX_TURRET_DEG && power > 0)) {
            power = 0;
        }

        turretMotor.setPower(power);

        // ===============================
        // TELEMETRY
        // ===============================
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("Robot Heading (deg)", robotHeadingDeg);
        telemetry.addData("Target Turret (deg)", targetTurretDeg);
        telemetry.addData("Turret Angle (deg)", currentTurretDeg);
        telemetry.addData("Error (deg)", errorDeg);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    // ===============================
    // HELPERS
    // ===============================

    private double getTurretAngleDeg() {
        double ticks = turretMotor.getCurrentPosition();
        return (ticks / (TICKS_PER_REV * GEAR_RATIO)) * 360.0;
    }

    private double wrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
