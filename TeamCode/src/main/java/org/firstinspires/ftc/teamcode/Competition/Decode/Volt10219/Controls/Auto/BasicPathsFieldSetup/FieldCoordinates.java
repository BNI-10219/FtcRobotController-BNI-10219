package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.BasicPathsFieldSetup;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue.BlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Autonomous(name = "Field Coordinate Testing", group = "Examples")
public abstract class FieldCoordinates extends BlueAlliance {

    Follower follower;

    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 72, Math.toDegrees(0));
    private final Pose ytestPose = new Pose(72, 72, Math.toDegrees(0));
    private final Pose xtestPose = new Pose(72, 98, Math.toDegrees(0));

    private PathChain yTest, xTest;

    public void buildPaths(){
        yTest = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, ytestPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ytestPose.getHeading())
                .build();

        xTest = follower.pathBuilder()
                .addPath(new BezierCurve(ytestPose, xtestPose))
                .setLinearHeadingInterpolation(ytestPose.getHeading(), xtestPose.getHeading())
                .build();
    }


    public void pathStates(){
        switch(pathState){
            case 0:
                follower.followPath(yTest);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(xTest);
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
        buildPaths();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {}
}
