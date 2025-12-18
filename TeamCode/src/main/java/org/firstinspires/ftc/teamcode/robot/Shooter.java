package org.firstinspires.ftc.teamcode.robot;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Outake utility class. Designed for simple static-style access from OpModes.
 * Call Outake.init(hardwareMap) once during OpMode init before using other methods.
 */
public class Shooter {
    private static ElapsedTime runtime = new ElapsedTime();
    private static DcMotorEx shooterR = null;
    private static DcMotorEx shooterL = null;

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
     * Initialize Outake hardware. Must be called once before using static methods.
     */
    public static void init(HardwareMap hardwareMap) {
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        // Set directions - adjust if motors spin the wrong way
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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


    public static void RunShooter() {
        if (shooterL != null && shooterR != null) {
            shooterL.setPower(PowerState.SHOOTER_RUN.power);
            shooterR.setPower(PowerState.SHOOTER_RUN.power);
        }
    }

    public static void SetShooterSpeed(double shooterSpeed) {
        double ticksPerSecond = (shooterSpeed * COUNTS_PER_REVOLUTION) / 60.0;

        shooterL.setVelocity(ticksPerSecond);
        shooterR.setVelocity(ticksPerSecond);
    }

    public static double GetCurrentRPM() {
        double currentTicksPerSecond = shooterL.getVelocity();

        return (currentTicksPerSecond * 60.0) / COUNTS_PER_REVOLUTION;
    }

    public static void SetShooterPower(double shooterPower) {
        if (shooterL != null && shooterR != null) {
            shooterL.setPower(shooterPower);
            shooterR.setPower(shooterPower);
        }
    }//asdfa

    public static void StopShooter() {
        if (shooterL != null && shooterR != null) {
            shooterL.setPower(PowerState.NOT_RUN.power);
            shooterR.setPower(PowerState.NOT_RUN.power);
        }
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
