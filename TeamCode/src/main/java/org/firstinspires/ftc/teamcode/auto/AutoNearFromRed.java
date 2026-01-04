package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AUTO: RED - Near Goal", group = "Autonomous")
public class AutoNearFromRed extends LinearOpMode {
    private Follower follower;
    private Timer actionTimer;
    private int pathState = 0;

    // --- 1. TUNABLE VARIABLES ---
    private final double SHOOT_DURATION   = 0.8; 
    private final double INTAKE_DURATION  = 1.2; 
    private final double PATH_END_DELAY   = 0.15; 

    // --- 2. POSES ---
    private final Pose startPose   = new Pose( 82, 9,   Math.toRadians(90));
    private final Pose shootPose   = new Pose( 82, 95,  Math.toRadians(0));

    private final Pose ctrlPose = new Pose(100, 90, Math.toRadians(0));
    private final Pose intake1Pose = new Pose(110, 84, Math.toRadians(0));
    private final Pose intake2Pose = new Pose(110, 108, Math.toRadians(0));
    private final Pose intake3Pose = new Pose(110, 132, Math.toRadians(0));
    private final Pose gatePose    = new Pose(110, 72, Math.toRadians(0));
    private final Pose parkPose    = new Pose(100, 72, Math.toRadians(0));

    // --- 3. PATHCHAINS ---
    private PathChain toShoot, toIntake1, toIntake2, toIntake3, toGate, toPark, intake1ToShoot;

    private void buildPaths() {
        // Linear path to initial shooting
        toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Curved path to Intake 1
        toIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, ctrlPose, intake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .build();

        intake1ToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(intake1Pose, ctrlPose, shootPose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shootPose.getHeading())
                .build();


        // Curved path to Intake 2
        toIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, ctrlPose, intake2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Curved path to Intake 3
        toIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2Pose, ctrlPose, intake3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Curved path back to Gate
        toGate = follower.pathBuilder()
                .addPath(new BezierCurve(intake3Pose, ctrlPose, gatePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), gatePose.getHeading())
                .build();

        // Final path to Park
        toPark = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, parkPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), parkPose.getHeading())
                .build();
    }

    // --- 4. MODULAR ACTION FUNCTIONS ---
    private void shoot() {
        // shooter.setPower(1.0);
    }

    private void intake() {
        // intake.setPower(1.0);
    }

    private void stopHardware() {
        // shooter.setPower(0);
        // intake.setPower(0);
    }

    /**
     * Centralized logic to handle arriving at a point, waiting for a settling period,
     * performing an action, and then moving to the next path.
     */
    private void handleAction(double actionDuration, PathChain nextPath, int nextState, Runnable action) {
        if (!follower.isBusy()) {
            double time = actionTimer.getElapsedTimeSeconds();
            
            // Wait for robot to settle before starting action
            if (time > PATH_END_DELAY) {
                action.run();
            }
            
            // Finish action and move to next state
            if (time > (PATH_END_DELAY + actionDuration)) {
                stopHardware();
                if (nextPath != null) follower.followPath(nextPath);
                setPathState(nextState);
            }
        } else {
            // Keep timer at zero until we actually arrive at the target Pose
            actionTimer.resetTimer();
        }
    }

    public void setPathState(int state) {
        pathState = state;
        actionTimer.resetTimer();
    }

    // @Override
    // public void loop() {
    //     follower.update();
    //     autonomousControl();
        
    //     telemetry.addData("Path State", pathState);
    //     telemetry.addData("X", follower.getPose().getX());
    //     telemetry.addData("Y", follower.getPose().getY());
    //     telemetry.update();
    // }

    // --- 5. STATE MACHINE ---
    private void autonomousControl() {
        switch (pathState) {
            case 0: // Move Start -> Shoot
                follower.followPath(toShoot);
                setPathState(1);
                break;

            case 1: // Shoot -> Intake 1
                handleAction(SHOOT_DURATION, toIntake1, pathState+1, this::shoot);
                break;

            case 2: // Intake 1 > Shoot
                handleAction(0, intake1ToShoot, pathState+1, this::shoot);

            case 3: // Shoot -> Intake 2
                handleAction(INTAKE_DURATION, toIntake2, 6, this::intake);
                break;

            case 4: // Intake 2 -> Intake 3
                handleAction(INTAKE_DURATION, toIntake3, pathState+1, this::intake);
                break;

            case 5: // Intake 3 -> Gate
                handleAction(INTAKE_DURATION, toGate, pathState+1, this::intake);
                break;

            case 6: // Drive Gate -> Park
                if (!follower.isBusy()) {
                    follower.followPath(toPark);
                    setPathState(6);
                }
                break;

            case 7: // Final completion
                if (!follower.isBusy()) {
                    telemetry.addLine("Auto Complete!");
                }
                break;
        }
    }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        actionTimer = new Timer();


        buildPaths();
        pathState = 0;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            autonomousControl();
            telemetry.addData("State: ", pathState);
            telemetry.addData("X: ", follower.getPose().getX());
            telemetry.addData("Y: ", follower.getPose().getY());
            telemetry.addData("Heading: ", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
