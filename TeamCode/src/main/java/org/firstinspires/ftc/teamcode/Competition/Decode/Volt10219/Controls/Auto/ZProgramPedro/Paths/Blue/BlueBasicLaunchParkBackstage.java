package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.Paths.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.AutoMainProgram;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Autonomous(name = "Blue Basic Launch Park Backstage", group = "Blue")
public class BlueBasicLaunchParkBackstage extends AutoMainProgram {

    Follower follower;

    private PathState pathState = PathState.READY;

    private final Pose startPose = new Pose(117, 131, Math.toDegrees(-143));
    private final Pose launch = new Pose(85, 84, Math.toDegrees(45));
    private final Pose park = new Pose(90, 40, Math.toDegrees(90));

    private PathChain launchPath, parkPath;

    private void buildPaths(){
        launchPath = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, launch))
                .setLinearHeadingInterpolation(startPose.getHeading(), launch.getHeading())
                .build();

        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, park))
                .setLinearHeadingInterpolation(launch.getHeading(), park.getHeading())
                .build();
    }

    public enum PathState{
        LAUNCH,
        PARK,
        READY;
    }


    private void pathStates(){
        switch(pathState){
            case LAUNCH:
                follower.followPath(launchPath);
                pathState = PathState.PARK;
                break;
            case PARK:
                if(!follower.isBusy()) {
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
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void init_loop() {}

    public void start() {
        pathState = PathState.LAUNCH;
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

