package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.AutoMainProgram;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Basic 3 Artifact Launch Program")
public class Basic3ArtifactLaunchProgram extends AutoMainProgram {

    Follower follower;
    private Timer pathTimer, opmodeTimer;

    private PathState pathState = PathState.READY;

    private final Pose startPose = new Pose(120, 132, Math.toRadians(-45));
    private final Pose launchPoseOne = new Pose(84, 84, Math.toRadians(-45));
    private final Pose getArtifact = new Pose(60, 108, Math.toRadians(90));
    private final Pose launchPoseTwo = new Pose(84, 84, Math.toRadians(-45));


    private PathChain driveToLaunchOne, driveToArtifactOne, driveToLaunchTwo;


    public void buildPaths() {
        driveToLaunchOne = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, launchPoseOne))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPoseOne.getHeading())
                .build();
        driveToArtifactOne = follower.pathBuilder()
                .addPath(new BezierCurve(launchPoseOne, getArtifact))
                .setLinearHeadingInterpolation(launchPoseOne.getHeading(), getArtifact.getHeading())
                .build();
        driveToLaunchTwo = follower.pathBuilder()
                .addPath(new BezierCurve(getArtifact, launchPoseTwo))
                .setLinearHeadingInterpolation(getArtifact.getHeading(), launchPoseTwo.getHeading())
                .build();
    }


    public enum PathState{
        LAUNCH_ONE,
        GET_ARTIFACT,
        LAUNCH_TWO,
        READY;
    }


    public void autonomousPathUpdate(){
        switch(pathState){
            case LAUNCH_ONE:
                follower.followPath(driveToLaunchOne);
                pathState = PathState.GET_ARTIFACT;
                break;
            case GET_ARTIFACT:
                if(!follower.isBusy()) {
                    follower.followPath(driveToArtifactOne);
                    pathState = PathState.LAUNCH_TWO;
                }
                break;
            case LAUNCH_TWO:
                if(!follower.isBusy()){
                    follower.followPath(driveToLaunchTwo);
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
        pathState = PathState.LAUNCH_ONE;
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {}
}
