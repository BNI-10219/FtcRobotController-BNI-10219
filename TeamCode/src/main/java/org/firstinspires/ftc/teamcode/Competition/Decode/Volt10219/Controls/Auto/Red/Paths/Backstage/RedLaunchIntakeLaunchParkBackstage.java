package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Backstage;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Autonomous(name = "Red Launch Intake Launch Park Backstage")
public class RedLaunchIntakeLaunchParkBackstage extends RedAlliance {

    Follower follower;

    private PathState pathState = PathState.READY;

    public DecodeBot Bot = new DecodeBot();

    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(120, 132, Math.toRadians(215));
    private final Pose launch = new Pose(84, 84, Math.toRadians(225));
    private final Pose intake = new Pose(108, 36, Math.toRadians(0));
    private final Pose intakePickup = new Pose(36, 128, Math.toRadians(90));
    private final Pose launchTwoPull = new Pose(72, 48, 157);
    private final Pose launchTwo = new Pose(84, 84, 225);
    private final Pose park = new Pose(108, 36, Math.toRadians(0));

    private Path launchOne;
    private PathChain intakePath, intakePickupPath, launchTwoPath, parkPath;

    private enum PathState {
        LAUNCHONE,
        INTAKE,
        INTAKEPICKUP,
        LAUNCHTWO,
        PARK,
        READY;
    }

    private void buildPaths() {
        launchOne = new Path(new BezierCurve(startPose, launch));
        launchOne.setLinearHeadingInterpolation(startPose.getHeading(), launch.getHeading());

        intakePath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, intake))
                .setLinearHeadingInterpolation(launch.getHeading(), intake.getHeading())
                .build();
        intakePickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(intake, intakePickup))
                .setLinearHeadingInterpolation(intake.getHeading(), intakePickup.getHeading())
                .build();
        launchTwoPath = follower.pathBuilder()
                .addPath(new BezierCurve(intakePickup, launchTwoPull, launchTwo))
                .setLinearHeadingInterpolation(intakePickup.getHeading(), launch.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(launchTwo, park))
                .setLinearHeadingInterpolation(launch.getHeading(), park.getHeading())
                .build();

    }

    public void pathState() {
        switch (pathState) {
            case LAUNCHONE:
                follower.followPath(launchOne);
                //launchV();
                pathState = PathState.INTAKE;
                break;
            case INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(intakePath);
                    pathState = PathState.INTAKEPICKUP;
                }
                break;
            case INTAKEPICKUP:
                Bot.driveForward(.25, 1);
                pathState = PathState.LAUNCHTWO;
                break;
            case LAUNCHTWO:
                if (!follower.isBusy()) {
                    follower.followPath(launchTwoPath);
                    //launchV();
                    pathState = PathState.PARK;
                }
                break;
            case PARK:
                if (!follower.isBusy()) {
                    follower.followPath(parkPath);
                    pathState = PathState.READY;
                }
                break;
            case READY:
                break;
        }
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.getHeading();
    }


    public void init_loop() {}

    public void start() {
        opmodeTimer.resetTimer();
        pathState = PathState.LAUNCHONE;
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        pathState();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {}
}


