package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Turret {
    private static double maxVelocity = 500;     // ticks per second
    private static double maxAcceleration = 300; // ticks per second²
    private static double profileVelocity = 0;
    private static double profilePosition = 0;


    // Hardware
    private static DcMotor turretMotor = null;

    private static DcMotorEx encoderPort = null; // driveFR port used for encoder

    // Gear ratio calculations
    private static final double DRIVE_PULLEY_TEETH = 24.0;
    private static final double TURRET_TEETH = 125.0;
    private static final double GEAR_RATIO = TURRET_TEETH / DRIVE_PULLEY_TEETH; // 5.208333

    // Motor specifications
    private static final double MOTOR_RPM = 435.0;
    private static final double MOTOR_TICKS_PER_REV = 384.5; // GoBilda 435 RPM Yellow Jacket

    // Turret angle calculations
    private static final double TICKS_PER_TURRET_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // PID constants - tune these for your robot
    private static double kP = 0.01;
    private static double kI = 0.0000;
    private static double kD = 0.0008;

    private static double lastError = 0;
    private static double integralSum = 0;
    private static long lastUpdateTime = 0;
    static ElapsedTime timer = new ElapsedTime();

    // Goal position (field coordinates in inches)
    // MODIFY THESE VALUES for your field setup
    private static double goalX = 28.0;  // X position of goal 64.0
    private static double goalY = 59.0; // Y position of goal (example: 6 feet out) -64.0

    // Turret offset from robot center (in inches)
    private static double turretOffsetX = 0.0;
    private static double turretOffsetY = 0.0;

    // Angle limits (in degrees) - adjust based on your physical constraints
    private static double minAngle = -180.0;
    private static double maxAngle = 180.0;

    private static double targetAngle = 0.0;

    /**
     * Initialize the turret system
     * @param hardwareMap The hardware map from your OpMode
     */
    public static void init(HardwareMap hardwareMap) {
        // Initialize turret motor
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Use driveFR port for encoder reading
        encoderPort = hardwareMap.get(DcMotorEx.class, "driveFR");
        encoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastUpdateTime = System.currentTimeMillis();
    }

    /**
     * Call this when the turret is facing forward to zero the encoder
     * IMPORTANT: Position turret to face forward, then call this method!
     */
    public static void zeroTurret() {
        encoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        integralSum = 0;
        lastError = 0;
        targetAngle = 0;
    }

    /**
     * Set the goal position on the field
     * @param x X coordinate of goal (inches)
     * @param y Y coordinate of goal (inches)
     */
    public static void setGoalPosition(double x, double y) {
        goalX = x;
        goalY = y;
    }

    /**
     * Set the turret offset from robot center
     * @param x X offset (inches)
     * @param y Y offset (inches)
     */
    public static void setTurretOffset(double x, double y) {
        turretOffsetX = x;
        turretOffsetY = y;
    }

    /**
     * Set angle limits for the turret
     * @param min Minimum angle in degrees
     * @param max Maximum angle in degrees
     */
    public static void setAngleLimits(double min, double max) {
        minAngle = min;
        maxAngle = max;
    }

    /**
     * Set PID constants
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     */
    public static void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
    public static double turretPIDControl(double reference, double state){

        double error = reference - state;

        integralSum += error * timer.seconds();

        double derivative = (error - lastError) / timer.seconds();

        lastError = error;

        timer.reset();

        return (error * kP) + (derivative * kD) + (integralSum * kI);

    }
    /**
     * Get current turret angle in degrees
     * @return Current angle
     */
    public static double getCurrentAngle() {
        int ticks = encoderPort.getCurrentPosition();
        return ticks / TICKS_PER_TURRET_DEGREE;
    }

    /**
     * Set target angle for the turret
     * @param angle Target angle in degrees (0 = forward)
     */
    public static void setTargetAngle(double angle) {
        targetAngle = Range.clip(angle, minAngle, maxAngle);
    }
    // Define a 'deadzone' so the motor doesn't hum when it's nearly perfect
    private final int DEADZONE = 5;
    // Gain (P-value): Adjust this to make the turret move faster or slower
    private final double Kp = 0.005;

    public void trackObject(HuskyLens.Block target, DcMotor turretMotor) {
        // 1. Calculate how many pixels the object is from the center (160)
        int error = target.x - 160;

        // 2. Check if we are close enough to stop (Deadzone)
        if (Math.abs(error) <= DEADZONE) {
            turretMotor.setPower(0);
        } else {
            // 3. Calculate motor power based on the error
            // As error gets smaller, power gets smaller
            double power = error * Kp;

            // 4. Clip the power so it doesn't exceed -1.0 to 1.0
            power = Range.clip(power, -0.5, 0.5); // Limiting to 0.5 for safety

            turretMotor.setPower(power);
        }
    }



    /**
     * Calculate the angle needed to aim at the goal
     * @param robotPose Current robot pose from Pinpoint odometry
     * @return Turret angle needed to aim at goal
     */
    public static double calculateAngleToGoal(Pose2D robotPose) {
        // Calculate turret position on field
        double turretX = robotPose.getX(DistanceUnit.INCH) + turretOffsetX;
        double turretY = robotPose.getY(DistanceUnit.INCH) + turretOffsetY;

        // Calculate vector to goal
        double deltaX = goalX - turretX;
        double deltaY = goalY - turretY;

        // Calculate absolute angle to goal (in degrees)
        double absoluteAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

        // Calculate relative angle (turret angle relative to robot)
        double robotHeading = robotPose.getHeading(AngleUnit.DEGREES);
        double relativeAngle = absoluteAngle - robotHeading;

        // Normalize to -180 to 180
        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        return relativeAngle;
    }

    /**
     * Aim at the goal automatically using Pinpoint odometry
     * @param robotPose Current robot pose from Pinpoint odometry
     */
    public static void aimAtGoal(Pose2D robotPose) {
        double angleToGoal = calculateAngleToGoal(robotPose);
        setTargetAngle(angleToGoal);
    }



    // Clamp power

    /**
     * Check if turret is at target angle
     * @param tolerance Tolerance in degrees
     * @return true if within tolerance
     */
    public static boolean isAtTarget(double tolerance) {
        return Math.abs(targetAngle - getCurrentAngle()) < tolerance;
    }

    /**
     * Reset the turret encoder
     */
    public static void resetEncoder() {
        encoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        integralSum = 0;
        lastError = 0;
        targetAngle = 0;
    }

    /**
     * Manual control of turret power (for testing)
     * @param power Power from -1.0 to 1.0
     */
    public static void setManualPower(double power) {
        turretMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Get current error (difference between target and current angle)
     * @return Error in degrees
     */
    public static double getError() {
        return targetAngle - getCurrentAngle();
    }

    /**
     * Stop the turret
     */
    public static void stop() {
        turretMotor.setPower(0);
    }

    /**
     * Get the target angle
     * @return Target angle in degrees
     */
    public static double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Get distance to goal
     * @param robotPose Current robot pose from Pinpoint odometry
     * @return Distance to goal in inches
     */
    public static double getDistanceToGoal(Pose2D robotPose) {
        double turretX = robotPose.getX(DistanceUnit.INCH) + turretOffsetX;
        double turretY = robotPose.getY(DistanceUnit.INCH) + turretOffsetY;

        double deltaX = goalX - turretX;
        double deltaY = goalY - turretY;

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
}