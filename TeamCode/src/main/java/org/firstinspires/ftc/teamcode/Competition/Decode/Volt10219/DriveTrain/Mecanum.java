package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Mecanum {
    public DcMotor flMotor;
    public DcMotor frMotor;
    public DcMotor rlMotor;
    public DcMotor rrMotor;

    public DcMotor leftEncoder;
    public DcMotor centerEncoder;

    //Declare rest of mechanisms
    //public DcMotor ballIntake;

    public CRServo ballIntakeOne = null;
    public CRServo ballIntakeTwo = null;

    public Servo artifactPush = null;
    public DcMotorEx ballLaunchOne = null;
    public DcMotorEx ballLaunchTwo = null;



    //public Servo ballPush;

    public LinearOpMode LinearOp = null;

    public static final double TICKS_PER_ROTATION = 386.3;

    public IMU imu = null;
    public double headingTolerance = 0.0;
    public double currentHeading = 0;

    public Mecanum(){
    }

    public void setMotorRunModes(DcMotor.RunMode mode){
        flMotor.setMode(mode);
        frMotor.setMode(mode);
        rlMotor.setMode(mode);
        rrMotor.setMode(mode);
    }

    public double getHeading(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading(){
        imu.resetYaw();
    }

    public void stopMotors(){
        flMotor.setPower(0);
        frMotor.setPower(0);
        rlMotor.setPower(0);
        rrMotor.setPower(0);
    }

    public void driveForward(double speed){
        flMotor.setPower(speed);
        frMotor.setPower(speed);
        rlMotor.setPower(speed);
        rrMotor.setPower(speed);
    }

    public void driveBack(double speed){
        flMotor.setPower(-speed);
        frMotor.setPower(-speed);
        rlMotor.setPower(-speed);
        rrMotor.setPower(-speed);
    }

    public void turnLeft(double speed){
        flMotor.setPower(speed);
        frMotor.setPower(-speed);
        rlMotor.setPower(speed);
        rrMotor.setPower(-speed);
    }

    public void turnRight(double speed){
        flMotor.setPower(-speed);
        frMotor.setPower(speed);
        rlMotor.setPower(-speed);
        rrMotor.setPower(speed);

    }

    public void strafeLeft(double speed){
        flMotor.setPower(-speed);
        frMotor.setPower(speed);
        rlMotor.setPower(speed);
        rrMotor.setPower(-speed);
    }

    public void strafeRight(double speed){
        flMotor.setPower(speed);
        frMotor.setPower(-speed);
        rlMotor.setPower(-speed);
        rrMotor.setPower(speed);
    }

    public void driveForward(double speed, double rotations){
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(flMotor.getCurrentPosition()) < ticks && LinearOp.opModeIsActive())){
            driveForward(speed);
            LinearOp.telemetry.addData("FL Motor Ticks: ", flMotor.getCurrentPosition());
            LinearOp.telemetry.update();
        }
    }

    public void driveBack(double speed, double rotations){
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(flMotor.getCurrentPosition()) < ticks && LinearOp.opModeIsActive())){
            driveBack(speed);
            LinearOp.telemetry.addData("FL Motor Ticks:", flMotor.getCurrentPosition());
            LinearOp.telemetry.update();
        }
    }

    public void turnLeft(double speed, double rotations){
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while ((Math.abs(flMotor.getCurrentPosition())) < ticks && LinearOp.opModeIsActive()){
            turnLeft(speed);
            LinearOp.telemetry.addData("FL Motor Ticks: ", flMotor.getCurrentPosition());
            LinearOp.telemetry.update();
        }
    }

    public void turnRight(double speed, double rotations){
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while((Math.abs(flMotor.getCurrentPosition())) < ticks && LinearOp.opModeIsActive()){
            turnRight(speed);
            LinearOp.telemetry.addData("FL Motor Ticks:", flMotor.getCurrentPosition());
            LinearOp.telemetry.update();
        }
    }

    public void strafeRight(double speed, double rotations){
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while((Math.abs(flMotor.getCurrentPosition())) < ticks && LinearOp.opModeIsActive()){
            strafeRight(speed);
            LinearOp.telemetry.addData("FL Motor Ticks:", flMotor.getCurrentPosition());
            LinearOp.telemetry.update();
        }
    }

    public void strafeLeft(double speed, double rotations){
        double ticks = rotations * TICKS_PER_ROTATION;
        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_TO_POSITION);

        while((Math.abs(flMotor.getCurrentPosition())< ticks && LinearOp.opModeIsActive())){
            strafeLeft(speed);
            LinearOp.telemetry.addData("FL Motor Ticks:", flMotor.getCurrentPosition());
            LinearOp.telemetry.update();
        }
    }

    public void resetEncoders(){
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
