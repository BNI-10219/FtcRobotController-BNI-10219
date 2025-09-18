package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.DriveTrain.Mecanum;

public class DecodeBot extends Mecanum {
    public HardwareMap hwBot = null;

    //servos and mechanisms

    public DecodeBot(){
    }

    public void initRobot(HardwareMap hwBot){

        flMotor = hwBot.dcMotor.get("front_Left_motor");
        frMotor = hwBot.dcMotor.get("front_right_motor");
        rlMotor = hwBot.dcMotor.get("rear_left_motor");
        rrMotor = hwBot.dcMotor.get("rear_right_motor");

        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        rlMotor.setDirection(DcMotor.Direction.FORWARD);
        rrMotor.setDirection(DcMotor.Direction.FORWARD);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ballPush = hwBot.servo.get("ball_push");
        ballPush.setDirection(Servo.Direction.FORWARD);

        //IMU for Control Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hwBot.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //init mechanisms
    }

    //mechanism methods
    public void launchBall(){
        ballLaunch.setPower(1);
    }
    public void intakeBall(){
        ballIntake.setPower(1);
    }
    public void outtakeBall(){
        ballIntake.setPower(-1);
    }

    public void pushBall(){
        ballPush.setPosition(1);
    }
    public void resetBallPush(){
        ballPush.setPosition(.1);
    }

    //make sure to make encoder versions

}
