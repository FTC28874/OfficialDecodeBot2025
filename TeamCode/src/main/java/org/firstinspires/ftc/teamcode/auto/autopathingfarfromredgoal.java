package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Shooter;

@Autonomous(name = "Far From Red Goal", group = "Linear OpMode")
public class autopathingfarfromredgoal extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private ElapsedTime actionTimer = new ElapsedTime();

    public enum PathState {
        START_TO_SHOOT,
        SHOOT_BALLS,
        GOTO_INTAKE_SET2,
        INTAKE2_TO_SHOOT,
        PARK,
    }

    PathState pathState;
    private final double start_x = 88;
    private final double start_y = 9;
    private final double shoot_x = start_x;
    private final double shoot_y = 95;
    private final double gate_x = 110;
    private final double gate_y = 66;
    private final double ready_intake_x = start_x;
    private final double do_intake_x = ready_intake_x + 30;
    private final double set_1_y = 84;
    private final double set_2_y = set_1_y - 24;
    private final double set_3_y = set_2_y - 24;

    private final Pose startPose = new Pose(start_x, start_y, Math.toRadians(90));
    private final Pose shootPose = new Pose(shoot_x, shoot_y, Math.toRadians(0));
    private final Pose intakeSet2 = new Pose(shoot_x, shoot_y, Math.toRadians(0));


    private PathChain startToShoot,
            goToIntake2,
            goToShoot;

    public void buildAllPaths() {
        startToShoot = buildPath(startPose, shootPose);
        goToIntake2 = buildPath(shootPose, intakeSet2);
        goToShoot = buildPath(intakeSet2, shootPose);
    }

    public PathChain buildPath(Pose p1, Pose p2) {
        return follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
                .build();
    }

    public void setNextPathState(PathState newState) {
        pathState = newState;
        if (pathTimer != null)
            pathTimer.resetTimer();
    }

    private void performShooting() {
        // TODO
    }

    private void performIntake() {
        // TODO
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START_TO_SHOOT:
                follower.followPath(startToShoot, true);
                setNextPathState(PathState.SHOOT_BALLS);
                break;
            case SHOOT_BALLS:
                performShooting();
                if (actionTimer.seconds() > 1.2) setPathState(PathState.GOTO_INTAKE_SET2);
                break;
            case GOTO_INTAKE_SET2:
                follower.followPath(goToIntake2, true);
                setPathState(PathState.INTAKE2_TO_SHOOT);
                break;
            case INTAKE2_TO_SHOOT:
                follower.followPath(goToShoot, true);
                setPathState(PathState.PARK);
                break;
            default:
                break;
        }
    }

    @Override
    public void runOpMode() {

        Shooter.init(hardwareMap);

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        buildPaths();
        pathState = PathState.START_TO_SHOOT;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            statePathUpdate();
            telemetry.addData("State", pathState);
            telemetry.addData("X Position", follower.getPose().getX());
            telemetry.addData("Y Position", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
