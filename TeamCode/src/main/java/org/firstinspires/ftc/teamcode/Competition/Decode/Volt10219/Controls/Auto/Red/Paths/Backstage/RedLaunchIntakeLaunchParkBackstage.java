package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Backstage;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.Tester.AngleCoordinatesProgram;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Competition.Z20220223PowerPlay.controls.Autonomus.Test.GyroDriveTest.TagDetection;

@Autonomous(name = "Red Launch Intake Launch Park Backstage")
public class RedLaunchIntakeLaunchParkBackstage extends RedAlliance {

    Follower follower;

    private PathState pathState = PathState.READY;

    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(132, 108, Math.toRadians(-135));
    private final Pose launch = new Pose(84, 84, Math.toRadians(-135));
    private final Pose intake = new Pose(36, 104, Math.toRadians(90));
    private final Pose intakePickup = new Pose(36, 128, Math.toRadians(90));
    private final Pose launchTwoPull = new Pose(72, 48);
    private final Pose park = new Pose(36, 108, Math.toRadians(90));

    private PathChain launchOne, intakePath, intakePickupPath, launchTwo, parkPath;

    private enum PathState {
        LAUNCHONE,
        INTAKE,
        INTAKEPICKUP,
        LAUNCHTWO,
        PARK,
        READY;
    }

    private void buildPaths() {
        launchOne = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, launch))
                .setLinearHeadingInterpolation(startPose.getHeading(), launch.getHeading())
                .build();
        intakePath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, intake))
                .setLinearHeadingInterpolation(launch.getHeading(), intake.getHeading())
                .build();
        intakePickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(intake, intakePickup))
                .setLinearHeadingInterpolation(intake.getHeading(), intakePickup.getHeading())
                .build();
        launchTwo = follower.pathBuilder()
                .addPath(new BezierCurve(intakePickup, launchTwoPull, launch))
                .setLinearHeadingInterpolation(intakePickup.getHeading(), launch.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, park))
                .setLinearHeadingInterpolation(launch.getHeading(), park.getHeading())
                .build();

    }

    public void pathState() {
        switch (pathState) {
            case LAUNCHONE:
                follower.followPath(launchOne);
                pathState = PathState.INTAKE;
                break;
            case INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(intakePath);
                    pathState = PathState.INTAKEPICKUP;
                }
                break;
            case INTAKEPICKUP:
                if (!follower.isBusy()) {
                    follower.followPath(intakePickupPath);
                    pathState = PathState.LAUNCHTWO;
                }
                break;
            case LAUNCHTWO:
                if (!follower.isBusy()) {
                    follower.followPath(launchTwo);
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


