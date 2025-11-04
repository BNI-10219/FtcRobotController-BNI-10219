package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Backstage;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Autonomous(name = "Acker: Red Launch Intake Launch Park Backstage")
public class RedLaunchIntakeLaunchParkBackstageAcker extends RedAlliance {

    Follower follower;

    private PathState pathState = PathState.READY;
    private LaunchState launchState = LaunchState.READY;

    private Timer opmodeTimer, intakeTimer, waitTimer, pathTimer;

    private final Pose startPose = new Pose(120, 132, Math.toRadians(215));
    private final Pose launch = new Pose(84, 84, Math.toRadians(225));
    private final Pose intake = new Pose(108, 36, Math.toRadians(0));
    private final Pose intakePickup = new Pose(36, 128, Math.toRadians(90));
    private final Pose launchTwoPull = new Pose(72, 48, 157);
    private final Pose park = new Pose(108, 36, Math.toRadians(0));

    private Path launchOne;
    private PathChain intakePath, intakePickupPath, launchTwoPath, parkPath;

    boolean scoringDone = false;

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
                .addPath(new BezierCurve(intakePickup, launchTwoPull, launch))
                .setLinearHeadingInterpolation(intakePickup.getHeading(), launch.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, park))
                .setLinearHeadingInterpolation(launch.getHeading(), park.getHeading())
                .build();

    }

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        intakeTimer = new Timer();
        intakeTimer.resetTimer();
        waitTimer = new Timer();
        waitTimer.resetTimer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        pathTimer = new Timer();
        pathTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.getHeading();
    }


    public void init_loop() {}

    public void start() {
        opmodeTimer.resetTimer();
        pathState = PathState.DRIVETOLAUNCH;
        launchState = LaunchState.READY;
        launchZone = LaunchZone.NONE;
    }

    public enum LaunchState {OUTTAKE, INTAKEONE, WAITONE, INTAKETWO, WAITTWO, INTAKETHREE, WAIT, READY, IDLE}
    public enum PathState{DRIVETOLAUNCH, LAUNCH, INTAKE, PICKUP, LAUNCHPOSTWO, LAUNCHTWO, PARK, READY, WAIT;}
    @Override
    public void loop() {

        autoPathing();
        automaticLaunch();

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        // Feedback to Driver Hub for debugging
        telemetry.addData("launch state", launchState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Is Busy: ", follower.isBusy());
        telemetry.update();
    }

    public void stop() {}

    public void autoPathing() {
        switch(pathState){
            case DRIVETOLAUNCH:
                follower.followPath(launchOne, true);
                pathState = PathState.LAUNCH;
                pathTimer.resetTimer();
                break;

            case LAUNCH:

               if (launchState == LaunchState.READY || launchState == LaunchState.IDLE ) {
                   launchState = LaunchState.OUTTAKE;
               }

                if (scoringDone) {
                    waitTimer.resetTimer();
                    pathState = PathState.PARK;
                }
                break;

            case PARK:
                follower.followPath(parkPath);
                if(!follower.isBusy()) {
                    pathState = PathState.READY;
                }
                break;
            case READY:
                break;
        }
    }

    public void automaticLaunch() {
        switch(launchState) {
            case READY:
                Bot.ballLaunchV();
                break;

            case OUTTAKE:
                Bot.ballOuttake();
                Bot.artifactPushDown();
                waitTimer.resetTimer();
                intakeTimer.resetTimer();
                launchState = LaunchState.WAIT;
                break;
            case WAIT:
                if (waitTimer.getElapsedTimeSeconds() > 3) {
                    intakeTimer.resetTimer();
                    Bot.intakeStop();
                    launchState = LaunchState.INTAKEONE;
                }
                break;
            case INTAKEONE:
                Bot.ballIntake();
                if (intakeTimer.getElapsedTimeSeconds() > 3) {
                    Bot.intakeStop();
                    waitTimer.resetTimer();
                    launchState = LaunchState.READY;
                    scoringDone = true;
                }
                break;
            case IDLE:
                break;

        }


    }


}


