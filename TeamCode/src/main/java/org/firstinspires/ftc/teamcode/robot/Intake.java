package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Intake utility class.
 * Call Intake.init(hardwareMap) once during OpMode init before using methods.
 */
public class Intake {
    private static ElapsedTime runtime = new ElapsedTime();
    private static DcMotor intake = null;
    private static Servo intakeServo = null;

    /**
     * Initialize Shooter hardware. Must be called once before using static methods.
     * @param hardwareMap
     */
    public static void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public enum PowerState {
        RUN(1.0),
        NOT_RUN(0.0),
        REVERSE(-1.0);
        private final double power;
        PowerState(double power) {
            this.power = power;
        }
    }

    /**
     * Enums for the servo controlling the Intake angle.
     */
    public enum AngleState {
        REST(0.4),
        DOWN(0.36);
        private final double angle;
        AngleState(double angle) {
            this.angle = angle;
        }
    }

    /**
     * Runs the intake.
     */
    public static void runIntake() {
        if (intake != null) intake.setPower(PowerState.RUN.power);
    }

    /**
     * Stops the intake
     */
    public static void stopIntake() {
        if (intake != null) intake.setPower(PowerState.NOT_RUN.power);
    }

    /**
     * Runs the intake backward.
     */
    public static void reverseIntake() {
        if (intake != null) intake.setPower(PowerState.REVERSE.power);
    }

    /**
     * Lowers the entire intake using the servo.
     */
    public static void lowerIntake() {
        intakeServo.setPosition(AngleState.DOWN.angle);
    }

    /**
     * Raises the intake using the servo.
     */
    public static void raiseIntake() {
        intakeServo.setPosition(AngleState.REST.angle);
    }

    /**
     * Sets the intake to a certain position using the servo
     * @param position value ranging from 0.0 to 1.0
     */
    public static void setIntakeToPosition(double position) {
        intakeServo.setPosition(position);
    }

    public static double getPower() {
        return intake.getPower();
    }

}
