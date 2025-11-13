package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Audience;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;
@Disabled
@Autonomous(name = "Tester: Red Launch Park Audience Cam Acker")
public class RedLaunchParkAudienceCamAcker extends RedAlliance {

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


    //                90 degrees
    //                     |
    //                     |
    //  180 degrees  --------------   0 degrees
    //                     |
    //                     |
    //                180 degrees

    Follower follower;

    protected int shotCount = 0;

    private PathState pathState = PathState.READY;
    private LaunchState launchState = LaunchState.READY;

    protected Timer creepTimer;
    protected static final double CREEP_POWER = 0.7;
    protected static final double CREEP_TIMEOUT_S = 2.5;

    private Timer opmodeTimer, intakeTimer, waitTimer, pathTimer, outtakeTimer;

    private final Pose startPose = new Pose(96, 8, Math.toRadians(270));
    private final Pose launch = new Pose(86, 12, Math.toRadians(248));
    private final Pose intake = new Pose(96, 34, Math.toRadians(2.5));
    private final Pose intakePickupEnd = new Pose(112, 34, Math.toRadians(2.5));
    private final Pose park = new Pose(96, 24, Math.toRadians(0));

    private Path launchOne;
    private PathChain intakePath, intakePickupPath, launchTwoPath, parkPath;

    boolean scoringDone = false;

    //public int motifID;

//    double targetTX = 23;
//    double targetTA = 3;
//    double llTolerance = 1.25;

    private void buildPaths() {
        launchOne = new Path(new BezierCurve(startPose, launch));
        launchOne.setLinearHeadingInterpolation(startPose.getHeading(), launch.getHeading());

        intakePath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, intake))
                .setLinearHeadingInterpolation(launch.getHeading(), intake.getHeading())
                .build();
        intakePickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(intake, intakePickupEnd))
                .setLinearHeadingInterpolation(intake.getHeading(), intakePickupEnd.getHeading())
                .setGlobalDeceleration()
                .build();
        launchTwoPath = follower.pathBuilder()
                .addPath(new BezierCurve(intakePickupEnd, launch))
                .setLinearHeadingInterpolation(intakePickupEnd.getHeading(), launch.getHeading())
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

        creepTimer = new Timer();
        creepTimer.resetTimer();


    }


    public void init_loop() {}

    public void start() {
        opmodeTimer.resetTimer();
        //limelight.start();
        pathState = PathState.DRIVETOLAUNCH;
        launchState = LaunchState.IDLE;
        launchZone = LaunchZone.NONE;
        scoringDone = false;
        shotCount = 0;
    }

    public enum LaunchState {OUTTAKE, WAIT, READY, IDLE}
    public enum PathState{DRIVETOLAUNCH, LAUNCH, INTAKE_START, PICKUP, INTAKE_PICKUP, LAUNCHPOSTWO, LAUNCHTWO, DECIDE, INTAKE_ARTIFACTS, PARK, READY, WAIT;}
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
        telemetry.addData("Path State: ", pathState);
        telemetry.update();
    }

    public void stop() {}

    //DOES NOT NEED TO BE CHANGED
    public void autoPathing() {
        switch(pathState){
            case DRIVETOLAUNCH:
                follower.followPath(launchOne, true);
                pathState = PathState.LAUNCH;
                launchState = LaunchState.READY;
                pathTimer.resetTimer();
                break;

            case LAUNCH:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchState == LaunchState.READY && !scoringDone ) {
                         launchState = LaunchState.OUTTAKE;
                    }
                }
                if (scoringDone) {
                    Bot.artifactPushUps();
                    Bot.ballIntake();
                    follower.followPath(intakePath);
                    waitTimer.resetTimer();
                    pathState = PathState.INTAKE_START;
                    pathTimer.resetTimer();

                }
                break;

            case INTAKE_START:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3){
                    creepTimer.resetTimer();
                    scoringDone = false;
                    follower.followPath(intakePickupPath);
                    pathState = PathState.INTAKE_PICKUP;
                    Bot.ballIntake();
                    pathTimer.resetTimer();
                }
                break;
            case INTAKE_PICKUP:
                if(!follower.isBusy()){
                    follower.followPath(launchTwoPath, true);
                    pathState = PathState.LAUNCHPOSTWO;
                    launchState = LaunchState.READY;
                    scoringDone = false;
                    pathTimer.resetTimer();
                }

            case INTAKE_ARTIFACTS:
                driveForwardCreep(CREEP_POWER);
                if (creepTimer.getElapsedTimeSeconds() >= CREEP_TIMEOUT_S) {
                    stopCreepDrive();
                    //Bot.intakeStop();
                    waitTimer.resetTimer();

                    follower.followPath(launchTwoPath, true);
                    pathState = PathState.LAUNCHPOSTWO;
                    launchState = LaunchState.READY;
                    scoringDone = false;
                    pathTimer.resetTimer();
                }
                break;

            case LAUNCHPOSTWO:

                if ( !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchState == LaunchState.READY && !scoringDone ) {
                        Bot.intakeStop();
                        launchState = LaunchState.OUTTAKE;
                    }
                }

                if (scoringDone) {
                    Bot.artifactPushUps();
                    Bot.ballIntake();
                    follower.followPath(parkPath);
                    waitTimer.resetTimer();
                    pathState = PathState.PARK;
                    pathTimer.resetTimer();

                }
                break;

            case PARK:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    pathState = PathState.READY;
                    pathTimer.resetTimer();
                    scoringDone = true;
                    launchState = LaunchState.IDLE;
                    Bot.intakeStop();

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
                Bot.ballLaunchBackField();
                outtakeTimer.resetTimer();
                intakeTimer.resetTimer();
                break;

            case OUTTAKE:
                //Bot.ballOuttake();
                Bot.ballLaunchAutoBack();
                Bot.artifactPushDown();
                if (outtakeTimer.getElapsedTimeSeconds() > 1.5) {
                    Bot.artifactPushDown();
                    waitTimer.resetTimer();
                    intakeTimer.resetTimer();
                    launchState = LaunchState.WAIT;
                    Bot.intakeStop();
                }
                break;

            case WAIT:
                Bot.ballIntake();//intake sooner
                if (waitTimer.getElapsedTimeSeconds() > 1) {
                    intakeTimer.resetTimer();
                    scoringDone = true;
                    shotCount ++;
                    launchState = LaunchState.IDLE;
                    Bot.artifactPushUps();
                }
                break;

            case IDLE:
                Bot.ballLaunchOne.setPower(0);
                Bot.ballLaunchTwo.setPower(0);
                break;


        }
    }


    protected void driveForwardCreep(double power) {

        Bot.flMotor.setPower(power);
        Bot.frMotor.setPower(power);
        Bot.rlMotor.setPower(power);
        Bot.rrMotor.setPower(power);
    }

    protected void driveBackwardCreep(double power) {
        Bot.flMotor.setPower(-power);
        Bot.frMotor.setPower(-power);
        Bot.rlMotor.setPower(-power);
        Bot.rrMotor.setPower(-power);
    }

    protected void stopCreepDrive() {
        Bot.flMotor.setPower(0);
        Bot.frMotor.setPower(0);
        Bot.rlMotor.setPower(0);
        Bot.rrMotor.setPower(0);

    }

}


