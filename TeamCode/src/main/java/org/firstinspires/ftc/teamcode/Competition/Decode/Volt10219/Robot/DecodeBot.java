package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.DriveTrain.Mecanum;

public class DecodeBot extends Mecanum {
    public HardwareMap hwBot = null;


    public double velocity = 500;
    public double velocity_low = 200;
    public double velocity_med = 1200;
    public double velocity_high = 2300;

    public DecodeBot() {
    }

    public void initRobot(HardwareMap hwBot) {

        flMotor = hwBot.dcMotor.get("front_Left_motor"); // CH Port 0
        frMotor = hwBot.dcMotor.get("front_right_motor"); // CH Port 1
        rlMotor = hwBot.dcMotor.get("rear_left_motor"); // CH Port 2
        rrMotor = hwBot.dcMotor.get("rear_right_motor"); // CH Port 3

        flMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        rlMotor.setDirection(DcMotor.Direction.REVERSE);
        rrMotor.setDirection(DcMotor.Direction.FORWARD);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ballIntakeOne = hwBot.get(CRServo.class, "intake_one");
        ballIntakeOne.setDirection(CRServo.Direction.FORWARD);

        ballIntakeTwo = hwBot.get(CRServo.class, "intake_two");
        ballIntakeTwo.setDirection(CRServo.Direction.REVERSE);

        ballLaunchOne = hwBot.get(DcMotorEx.class, "ball_launch_one");
        ballLaunchOne.setDirection(DcMotorSimple.Direction.FORWARD);
        ballLaunchOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ballLaunchOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ballLaunchTwo = hwBot.get(DcMotorEx.class, "ball_launch_two");
        ballLaunchTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        ballLaunchTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ballLaunchTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        artifactPush = hwBot.servo.get("artifact_push");
        artifactPush.setDirection(Servo.Direction.FORWARD);

        //IMU for Control Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hwBot.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //init mechanisms
    }

    public void ballOuttake() {
        //ballIntake.setPower(0.5);

        ballIntakeOne.setPower(-1);
        ballIntakeTwo.setPower(-1);
    }

    public void ballIntake() {
        ballIntakeOne.setPower(1);
        ballIntakeTwo.setPower(1);
    }
    public void ballIntakeHalf() {
        ballIntakeOne.setPower(0.5);
        ballIntakeTwo.setPower(0.5);
    }

    public void intakeStop(){
        ballIntakeOne.setPower(0);
        ballIntakeTwo.setPower(0);
    }


    public void ballLaunchV() {
        velocity = velocity_med;
        velocity = Range.clip(velocity, 0, 5000);
        ballLaunchOne.setVelocity(velocity);
        ballLaunchTwo.setVelocity(velocity);

//        ballLaunchTwo.setPower(0.425);
//        ballLaunchOne.setPower(0.425);
    }

    public void ballLaunchMidV(){
        velocity = velocity_low;
        velocity = Range.clip(velocity, 0, 5000);
        ballLaunchOne.setVelocity(velocity);
        ballLaunchTwo.setVelocity(velocity);

//        ballLaunchTwo.setPower(0.225);
//        ballLaunchOne.setPower(0.225);
    }

    public void ballLaunchBackField(){
        velocity = velocity_high;
        velocity = Range.clip(velocity, 0, 5000);
        ballLaunchOne.setVelocity(velocity);
        ballLaunchTwo.setVelocity(velocity);

//        ballLaunchTwo.setPower(0.625);
//        ballLaunchOne.setPower(0.625);
    }

    public void ballLaunchStop(){
        ballLaunchTwo.setPower(0);
        ballLaunchOne.setPower(0);
    }


    public void artifactPushReset(){
        artifactPush.setPosition(0);
    }
    public void artifactPushIntake(){
        artifactPush.setPosition(0.5);
    }

//    public void ballIntakeStop() {
//        ballIntake.setPower(0);
//    }

//    public void ballLaunch(double speed, double rotations) {
//        double ticks = rotations * TICKS_PER_ROTATION;
//        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while ((Math.abs(ballLaunchOne.getCurrentPosition())) < ticks && ((Math.abs(ballLaunchTwo.getCurrentPosition()) < ticks) && LinearOp.opModeIsActive())) {
//            {
//                ballLaunch();
//            }
//        }
//    }

}
