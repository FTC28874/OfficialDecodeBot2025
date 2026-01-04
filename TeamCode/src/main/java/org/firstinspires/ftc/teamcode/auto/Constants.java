package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Constants for Pedro Pathing.
 * Optimized for Smoothness, Passive Braking, and High-Speed Transitions.
 */
public class Constants {

    // PID and Follower Tuning
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.07185)
            .forwardZeroPowerAcceleration(-31.108642686848118)
            .lateralZeroPowerAcceleration(-52.132651227924235)
            // Added D coefficient (0.02) to dampen stops and prevent jerking
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.02, 0))
            // Heading PID with D damping (0.15) to prevent rotation "snap"
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.15, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 1, 0.00001, 0.6, 0.01))
            .centripetalScaling(0.0007);

    // Drivetrain Configuration
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.75) // Increased overhead allows PID to smooth out movements
            .rightFrontMotorName("driveFR")
            .rightRearMotorName("driveBR")
            .leftRearMotorName("driveBL")
            .leftFrontMotorName("driveFL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(83.90387683778297)
            .yVelocity(63.115439107098915);

    // Pinpoint Localizer Settings
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.126)
            .strafePodX(-6.126)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // Path Constraints
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 0.6);

    /**
     * Initializes the Follower object with all custom constants.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    /**
     * Configures all drivetrain motors to BRAKE mode.
     * This ensures a "Passive Lock" when holdEnd is false.
     */
    public static void setBrakeMode(HardwareMap hardwareMap) {
        String[] motors = {"driveFL", "driveFR", "driveBL", "driveBR"};
        for (String name : motors) {
            try {
                DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                // Silently handle if motor is not found in hardwareMap
            }
        }
    }
}