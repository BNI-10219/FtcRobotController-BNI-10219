package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.DriveTrain.Mecanum;

public class ProgramBot extends Mecanum {
    public HardwareMap hwBot = null;

    //servos and mechanisms


    public ProgramBot(){
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
        ballIntakeOne.setDirection(CRServo.Direction.REVERSE);

        ballIntakeTwo = hwBot.get(CRServo.class, "intake_two");
        ballIntakeTwo.setDirection(DcMotorSimple.Direction.FORWARD);



        //IMU for Control Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hwBot.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //init mechanisms
    }

    public void ballIntake() {
        //ballIntake.setPower(0.5);

        ballIntakeOne.setPower(1);
        ballIntakeTwo.setPower(1);
    }

    public void ballOuttake() {
        //ballIntake.setPower(0.5);

        ballIntakeOne.setPower(-1);
        ballIntakeTwo.setPower(-1);
    }
}
