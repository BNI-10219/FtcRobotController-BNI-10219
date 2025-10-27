package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ProgramPedro.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue.BlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ProgramPedro.AutoMainProgram;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Autonomous(name = "Basic 3 Artifact Launch Program")
public class Basic3ArtifactLaunchProgram extends AutoMainProgram {

    Follower follower;

    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(9, 66, Math.toDegrees(180));
    private final Pose launchPose = new Pose(18, 66, Math.toDegrees(180));
    private final Pose anglePose = new Pose(18, 66, Math.toDegrees(90));

    private PathChain driveToPos, angle;

    public void buildPaths(){
        driveToPos = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();

        angle = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, anglePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), anglePose.getHeading())
                .build();
    }


    public void pathStates(){
        switch(pathState){
            case 0:
                follower.followPath(driveToPos);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(angle);
                    setPathState(2);
                    break;
                }
            case 2:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;
        }

    }

    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
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
        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        pathStates();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {}
}
