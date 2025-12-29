package org.firstinspires.ftc.teamcode.robot;



import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Shooter utility class. Designed for simple static-style access from OpModes.
 * Call Shooter.init(hardwareMap) once during OpMode init before using other methods.
 */
public class Shooter {
    private static ElapsedTime runtime = new ElapsedTime();
    private static DcMotorEx shooterU = null;
    private static DcMotorEx shooterD = null;
    private static Servo shooterServo = null;
    private static DcMotor turret = null;
    private static Servo stopperServo = null;

    // Shooter Control Variables
    private static final double COUNTS_PER_REVOLUTION = 28;

    private static final double SHOOTER_RPM_HIGH = 6000.0;
    private static final double SHOOTER_RPM_LOW = 100.0;
    private final static double Kp  = 0.001;     // Start small. Use to fix residual error.
    private final static double Ki  = 0.001;       // Start at 0. Use to eliminate steady-state error.
    private final static double Kd  = 0.0;       // Start at 0. Use to dampen overshoot/oscillations.
    private final static double Kf  = 0.0004;
    private static double integralSum = 0;
    private static double lastError = 0;


    /**
     * Initialize Shooter hardware. Must be called once before using static methods.
     */
    public static void init(HardwareMap hardwareMap) {
        shooterU = hardwareMap.get(DcMotorEx.class, "shooterU");
        shooterD = hardwareMap.get(DcMotorEx.class, "shooterD");

        turret = hardwareMap.get(DcMotor.class, "turret");

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        stopperServo = hardwareMap.get(Servo.class, "stopperServo");

        // Set directions - adjust if motors spin the wrong way
        shooterD.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterU.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterU.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    private enum PowerState {
        RUN(1.0),
        NOT_RUN(0.0),
        SHOOTER_RUN(0.5),
        REVERSE(-1.0);
        private final double power;
        PowerState(double power) {
            this.power = power;
        }
    }

    /**
     * Enums for the servo controlling the Shooter Hood.
     */
    public enum HoodState {
        UP(0.6),
        DOWN(0.1);
        public final double angle;
        HoodState(double angle) {
            this.angle = angle;
        }
    }

    /**
     * Enums for the Stopper Servo.
     */
    public enum StopperServoState {
        PASS(0.0),
        STOP(0.3);
        public final double angle;
        StopperServoState(double angle) {
            this.angle = angle;
        }
    }

    public static void setShooterSpeed(double shooterSpeed) {
        double ticksPerSecond = (shooterSpeed * COUNTS_PER_REVOLUTION) / 60.0;

        shooterD.setVelocity(ticksPerSecond);
        shooterU.setVelocity(ticksPerSecond);
    }

    /**
    * Gets the current shooter motor RPM using the Encoders.
     */
    public static double getCurrentRPM() {
        double currentTicksPerSecond = shooterD.getVelocity();

        return (currentTicksPerSecond * 60.0) / COUNTS_PER_REVOLUTION;
    }

    /**
     * Sets the shooter motor power level.
     * @param shooterPower Ranges from -1.0 to 1.0
     */
    public static void setShooterPower(double shooterPower) {
        if (shooterD != null && shooterU != null) {
            shooterD.setPower(shooterPower);
            shooterU.setPower(shooterPower);
        }
    }

    /**
     * Stops the shooter from running.
     */
    public static void stopShooter() {
        if (shooterD != null && shooterU != null) {
            shooterD.setPower(PowerState.NOT_RUN.power);
            shooterU.setPower(PowerState.NOT_RUN.power);
        }
    }

    /**
     * Raises the Shooter Hood using the servo.
     */
    public static void raiseShooter() {
        shooterServo.setPosition(HoodState.UP.angle);
    }

    /**
     * Lowers the Shooter Hood using the servo.
     */
    public static void lowerShooter() {
        shooterServo.setPosition(HoodState.DOWN.angle);
    }

    /**
     * Sets the shooter hood to a certain position.
     * Input ranges from 0.0 to 1.0.
     * @param shooterPosition
     */
    public static void setShooterPosition(double shooterPosition) {
        shooterServo.setPosition(shooterPosition);
    }

    /**
     * Main computational method for the PID in the shooter.
     * Returns a double ranging from -1.0 to 1.0
     *
     * @param reference
     * @param state
     * @return <code>Range.clip(output, -1.0, 1.0);</code> - Equals a double ranging from -1.0 to 1.0
     */
    public static double PIDControl(double reference, double state) {
        // 1. Calculate Error
        double error = reference - state;

        // Time since last loop iteration (Delta Time)
        double dt = runtime.seconds();
        runtime.reset();

        // 2. Integral Component (with anti-windup check)
        // Adjust '100.0' based on the magnitude of error you want to ignore
        if (Math.abs(error) < 100.0) {
            integralSum += error * dt;
        }

        // 3. Derivative Component
        double derivative = (error - lastError) / dt;
        lastError = error;

        // 4. Full PID + Feedforward Calculation
        double output = (error * Kp) +
                (integralSum * Ki) +
                (derivative * Kd) +
                (reference * Kf);

        // 5. Clamp Output Power
        return Range.clip(output, -1.0, 1.0);
    }

    public static void turnTurretDirection(boolean right, double turretPower) {
        if (right) {
            turret.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            turret.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        turret.setPower(turretPower);
    }

    public static void setStopperServoBlock() {
        stopperServo.setPosition(StopperServoState.STOP.angle);
    }
    public static void setStopperServoPass() {
        stopperServo.setPosition(StopperServoState.PASS.angle);
    }
}
