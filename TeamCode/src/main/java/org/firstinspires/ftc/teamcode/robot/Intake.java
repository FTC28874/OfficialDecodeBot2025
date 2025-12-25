package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Intake utility class. Call Intake.init(hardwareMap) once during OpMode init before using methods.
 */
public class Intake {
    private static ElapsedTime runtime = new ElapsedTime();
    private static DcMotor intake = null;
    private static Servo intakeServo = null;


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

    public enum AngleState {
        REST(0.4),
        DOWN(0.32);
        private final double angle;
        AngleState(double angle) {
            this.angle = angle;
        }
    }

    public static void runIntake() {
        if (intake != null) intake.setPower(PowerState.RUN.power);
    }

    public static void stopIntake() {
        if (intake != null) intake.setPower(PowerState.NOT_RUN.power);
    }

    public static void reverseIntake() {
        if (intake != null) intake.setPower(PowerState.REVERSE.power);
    }

    public static void lowerIntake() {
        intakeServo.setPosition(AngleState.DOWN.angle);
    }

    public static void raiseIntake() {
        intakeServo.setPosition(AngleState.REST.angle);
    }

    public static void setIntakeToPosition(double position) {
        intakeServo.setPosition(position);
    }

    public static double getPower() {
        return intake.getPower();
    }

}
