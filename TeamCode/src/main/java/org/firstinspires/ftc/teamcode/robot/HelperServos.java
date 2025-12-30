package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Helper Servo utility class. Designed for simple static-style access from other OpModes.
 * Call <code>HelperServos.init(hardwareMap)</code> once during OpMode init before using other methods.
 */
public class HelperServos {
    private static ElapsedTime runtime = new ElapsedTime();
    private static Servo stopperServo = null;
    private static Servo pusherServo = null;

    /**
     * Initialize Servo hardware. Must be called before using static methods.
     */
    public static void init(HardwareMap hardwareMap) {
        stopperServo = hardwareMap.get(Servo.class, "stopperServo");
        pusherServo = hardwareMap.get(Servo.class, "pusherServo");
    }

    private enum StopperState {
        PASS(0.0),
        BLOCK(0.5);
        private final double angle;
        StopperState(double angle) {
            this.angle = angle;
        }
    }
    private enum PusherState {
        REST(0.4),
        PUSH(0.0);
        private final double angle;
        PusherState(double angle) {
            this.angle = angle;
        }
    }

    public static void setPusherPosition(double position) {
        pusherServo.setPosition(position);
    }
    public static void setStopperPosition(double position) {
        stopperServo.setPosition(position);
    }

    public static void setPusherRest() {
        setPusherPosition(PusherState.REST.angle);
    }
    public static void setPusherPush() {
        setPusherPosition(PusherState.PUSH.angle);
    }

    public static void setStopperPass() {
        setStopperPosition(StopperState.PASS.angle);
    }
    public static void setStopperStop() {
        setStopperPosition(StopperState.BLOCK.angle);
    }


}
