package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;

//@Disabled
@TeleOp(name = "Velocity Tester", group="Tester")

public class VelocityTester extends OpMode {

//    public DecodeBot Bot = new DecodeBot();

    public DcMotorEx ballLaunchOne = null;
    public DcMotorEx ballLaunchTwo = null;

    public Servo artifactPush = null;

    public double velocity = 500;
    public double velocity_low = 200;
    public double velocity_med = 1200;
    public double velocity_high = 2300;
    public double incValue = 1;

    @Override
    public void init() {
//        Bot.initRobot(hardwareMap);

        ballLaunchOne = hardwareMap.get(DcMotorEx.class, "ball_launch_one");
        ballLaunchOne.setDirection(DcMotorSimple.Direction.REVERSE);
        ballLaunchOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ballLaunchOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ballLaunchTwo = hardwareMap.get(DcMotorEx.class, "ball_launch_two");
        ballLaunchTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        ballLaunchTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ballLaunchTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        artifactPush = hardwareMap.servo.get("artifact_push");
        artifactPush.setDirection(Servo.Direction.FORWARD);
    }


    @Override
    public void loop() {

//        if (gamepad1.a) {
//            velocity = velocity_low;
//        }
//
//        if(gamepad1.b){
//            velocity = velocity_med;
//        }
//
//        if(gamepad1.y){
//            velocity = velocity_high;
//        }
//
//        if(gamepad1.x){
//            velocity = 0;
//        }

        if (gamepad1.right_bumper) {
            velocity += incValue;
        }

        if (gamepad1.left_bumper) {
            velocity -= incValue;
        }


        //velocity = Range.clip(velocity, 0, 5000);
        ballLaunchOne.setVelocity(velocity);
        ballLaunchTwo.setVelocity(velocity);
        update_telemetry();

        artifactPushControl();
    }

    public void artifactPushControl(){

        if(gamepad1.b){
            artifactPush.setPosition(0.95);
        }
        if(gamepad1.a){
            artifactPush.setPosition(0.4825);
        }
        if(gamepad1.y){
            artifactPush.setPosition(0.6);
        }
    }

    public void update_telemetry () {
        telemetry.addData("left motor power: ", ballLaunchOne.getPower());
        telemetry.addData("left motor velocity: ", Math.abs(ballLaunchOne.getVelocity()));
        telemetry.addData("left motor encoders: ", ballLaunchOne.getCurrentPosition());
        telemetry.addLine("");
        telemetry.addData("Right motor power: ", ballLaunchTwo.getPower());
        telemetry.addData("Right motor velocity: ", Math.abs(ballLaunchTwo.getVelocity()));
        telemetry.addData("Right motor encoders: ", ballLaunchTwo.getCurrentPosition());
        telemetry.addData("Velocity: ", velocity);

    }

}
