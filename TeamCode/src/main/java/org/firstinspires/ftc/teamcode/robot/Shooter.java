package org.firstinspires.ftc.teamcode.robot;



import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Outake utility class. Designed for simple static-style access from OpModes.
 * Call Outake.init(hardwareMap) once during OpMode init before using other methods.
 */
public class Shooter {
    private static ElapsedTime runtime = new ElapsedTime();
    private static DcMotorEx shooterU = null;
    private static DcMotorEx shooterD = null;
    private static Servo shooterServo = null;

    // Shooter Control Variables
    private static final double COUNTS_PER_REVOLUTION = 28;

    private static final double SHOOTER_RPM_HIGH = 6000.0;
    private static final double SHOOTER_RPM_LOW = 100.0;
    private static double Kp  = 0.001;     // Start small. Use to fix residual error.
    private static double Ki  = 0.0;       // Start at 0. Use to eliminate steady-state error.
    private static double Kd  = 0.0;       // Start at 0. Use to dampen overshoot/oscillations.
    private static double Kf  = 0.0004;
    private static double integralSum = 0;
    private static double lastError = 0;


    /**
     * Initialize Shooter hardware. Must be called once before using static methods.
     */
    public static void init(HardwareMap hardwareMap) {
        shooterU = hardwareMap.get(DcMotorEx.class, "shooterU");
        shooterD = hardwareMap.get(DcMotorEx.class, "shooterD");

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        // Set directions - adjust if motors spin the wrong way
        shooterD.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterU.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterD.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterU.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public enum PowerState {
        RUN(1.0),
        NOT_RUN(0.0),
        SHOOTER_RUN(0.5),
        REVERSE(-1.0);
        private final double power;
        PowerState(double power) {
            this.power = power;
        }
    }

    public enum AngleState {
        UP(0.4),
        DOWN(0.2);
        public final double angle;
        AngleState(double angle) {
            this.angle = angle;
        }
    }

    public static void setShooterSpeed(double shooterSpeed) {
        double ticksPerSecond = (shooterSpeed * COUNTS_PER_REVOLUTION) / 60.0;

        shooterD.setVelocity(ticksPerSecond);
        shooterU.setVelocity(ticksPerSecond);
    }

    public static double getCurrentRPM() {
        double currentTicksPerSecond = shooterD.getVelocity();

        return (currentTicksPerSecond * 60.0) / COUNTS_PER_REVOLUTION;
    }

    public static void setShooterPower(double shooterPower) {
        if (shooterD != null && shooterU != null) {
            shooterD.setPower(shooterPower);
            shooterU.setPower(shooterPower);
        }
    }//asdfa

    public static void stopShooter() {
        if (shooterD != null && shooterU != null) {
            shooterD.setPower(PowerState.NOT_RUN.power);
            shooterU.setPower(PowerState.NOT_RUN.power);
        }
    }

    public static void raiseShooter() {
        shooterServo.setPosition(AngleState.UP.angle);
    }

    public static void lowerShooter() {
        shooterServo.setPosition(AngleState.DOWN.angle);
    }

    public static void setShooterPosition(double shooterPosition) {
        shooterServo.setPosition(shooterPosition);
    }

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
}
