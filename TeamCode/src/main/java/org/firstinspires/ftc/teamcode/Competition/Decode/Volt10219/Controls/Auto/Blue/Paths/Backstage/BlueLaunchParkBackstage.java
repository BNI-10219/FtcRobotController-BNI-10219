package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue.Paths.Backstage;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue.BlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Backstage.RedLaunchParkBackstage;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Autonomous(name = "Blue Launch Park Backstage")
public class BlueLaunchParkBackstage extends BlueAlliance {

    //   (0, 144)                          (144, 144)
    //      --------------------------------
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      ---------------------------------
    //   (0,0)                              (144, 0)

    Follower follower;

    private PathState pathState = PathState.READY;
    private LaunchState launchState = LaunchState.READY;

    private Timer opmodeTimer, intakeTimer, waitTimer, pathTimer, outtakeTimer;

    private final Pose startPose = new Pose(24, 132, Math.toRadians(315));
    private final Pose launch = new Pose(50, 76, Math.toRadians(303));
    private final Pose intake = new Pose(108, 36, Math.toRadians(180));//UNUSED POINT
    private final Pose intakePickup = new Pose(36, 128, Math.toRadians(180));//UNUSED POINT
    private final Pose launchTwoPull = new Pose(72, 48, Math.toRadians(157));//UNUSED POINT
    private final Pose park = new Pose(34, 36, Math.toRadians(0));

    private Path launchOne;
    private PathChain intakePath, intakePickupPath, launchTwoPath, parkPath;

    boolean scoringDone = false;

    //DOES NOT NEED TO BE CHANGED
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
        outtakeTimer = new Timer();
        outtakeTimer.resetTimer();
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

    public enum LaunchState {OUTTAKE, INTAKEONE, WAITONE, INTAKETWO, WAITTWO, INTAKETHREE, WAIT, READY, IDLE, OUTTAKEONE, OUTTAKETWO, OUTTAKETHREE}
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

    //DOES NOT NEED TO BE CHANGED
    public void autoPathing() {
        switch(pathState){
            case DRIVETOLAUNCH:
                follower.followPath(launchOne, true);
                pathState = PathState.LAUNCH;
                pathTimer.resetTimer();
                break;

            case LAUNCH:
                if ( !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchState == LaunchState.READY || launchState == LaunchState.IDLE) {
                        launchState = LaunchState.OUTTAKE;
                    }
                }
                if (scoringDone) {
                    waitTimer.resetTimer();
                    pathState = PathState.PARK;
                    pathTimer.resetTimer();
                }
                break;

            case PARK:
                follower.followPath(parkPath);
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                    pathState = PathState.READY;
                }
                break;
            case READY:
                break;
        }
    }

    //LAUNCHING CODE DURING AUTO - MIGHT NEED TO BE CHANGED
    public void automaticLaunch() {
        switch(launchState) {
            case READY:
                Bot.ballLaunchAutoV();//setting VELOCITY for launching 1st artifact
                outtakeTimer.resetTimer();
                break;

            case OUTTAKE:
                Bot.ballIntake();//actually launching
                if(outtakeTimer.getElapsedTimeSeconds() > .25){
                    Bot.intakeStop();
                    Bot.ballOuttake();
                }
                Bot.artifactPushAuto();
                waitTimer.resetTimer();
                intakeTimer.resetTimer();
                launchState = LaunchState.WAIT;
                break;
            case WAIT:
                if (waitTimer.getElapsedTimeSeconds() > 1) {
                    intakeTimer.resetTimer();
                    //Bot.intakeStop();
                    launchState = LaunchState.INTAKEONE;
                    Bot.ballLaunchV();//VELOCITY for launching 2nd artifact
                }
                break;
            case INTAKEONE:
                Bot.ballOuttake();
                Bot.artifactPushDown();//pushing the artifact into the launcher
                if (intakeTimer.getElapsedTimeSeconds() > 2) {
                    Bot.intakeStop();
                    waitTimer.resetTimer();
                    launchState = LaunchState.IDLE;
                    scoringDone = true;

                }
                break;
            case WAITONE:
                if (waitTimer.getElapsedTimeSeconds() > 1) {
                    intakeTimer.resetTimer();
                    Bot.intakeStop();
                    launchState = LaunchState.INTAKETWO;
                }
                break;
            case INTAKETWO:
                Bot.ballOuttake();
                if (intakeTimer.getElapsedTimeSeconds() > 1.5) {
                    Bot.intakeStop();
                    waitTimer.resetTimer();
                    launchState = LaunchState.IDLE;
                    scoringDone = true;   //***********************************************/////////////
                }
                break;
            case IDLE:
                break;

        }


    }


}


