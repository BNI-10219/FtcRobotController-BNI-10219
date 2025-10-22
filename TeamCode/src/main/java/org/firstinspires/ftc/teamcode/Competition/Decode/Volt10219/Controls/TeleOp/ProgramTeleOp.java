package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.ProgramBot;

@TeleOp(name = "Decode Program TeleOp")
public class ProgramTeleOp extends OpMode {
    double leftStickYVal;
    double leftStickXVal;
    double rightStickYVal;
    double rightStickXVal;

    double powerThreshold = 0;
    double speedMultiply = 1;

    double targetTX = 0;
    double targetTA = 20;
    double txTolerance = 9;
    double taTolerance = 4;

    double flSpeed;
    double frSpeed;
    double rlSpeed;
    double rrSpeed;

    public ProgramBot Bot = new ProgramBot();

    private boolean autoPosition = false;
    private Limelight3A limelight;

    @Override
    public void init(){
        Bot.initRobot(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
    }

    public void init_loop(){}

    public void start(){
        limelight.start();
    }

    @Override
    public void loop(){
        speedControl();
        //drive();
        telemetryOutput();
        fieldCentricDrive();
        intakeControl();
        autoPositioning();
    }

    public void autoPositioning(){
        autoPosition = gamepad1.a;
        if (!autoPosition){return;}

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addLine("No valid LL target");
            Bot.stopMotors();
            telemetry.update();
            return;
        }

        double txDifference = result.getTx() - targetTX;
        double taDifference = result.getTa() - targetTA;

        telemetry.addData("Tx Difference: ", txDifference);
        telemetry.addData("Ta Difference: ", taDifference);
        telemetry.addData("LL Ta:", result.getTa() );
        telemetry.addData("LL Tx: ", result.getTx());
        telemetry.update();



        if (autoPosition){
            if (txDifference <  -txTolerance){
                Bot.strafeRight(.5);
            }
            else if (txDifference > txTolerance){
                Bot.strafeLeft(.5);
            }
            else if (taDifference > taTolerance){
                Bot.driveForward(0.2);
            }
            else if (taDifference < -taTolerance){
                Bot.driveBack(.3);
            }
            else {
                Bot.stopMotors();
            }
        }
    }

    public void fieldCentricDrive(){
        double y;
        double x;
        double rx;

        y = Math.abs(Math.pow(gamepad1.left_stick_y, 2)); // Remember, Y stick value is reversed
        if (gamepad1.left_stick_y > 0) y *= -1;
        x = Math.abs(Math.pow(gamepad1.left_stick_x, 2));
        if (gamepad1.left_stick_x < 0) x *= -1;
        rx = Math.abs(Math.pow(gamepad1.right_stick_x, 2));
        if (gamepad1.right_stick_x < 0) rx *= -1;


        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if(gamepad1.options){
            Bot.imu.resetYaw();
        }
        if (gamepad1.options) {
            Bot.imu.resetYaw();
        }

        double botHeading = Bot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPower = (rotY + rotX + rx) / denominator;
        double rlPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY - rotX - rx) / denominator;
        double rrPower = (rotY + rotX - rx) / denominator;

        Bot.flMotor.setPower(flPower);
        Bot.frMotor.setPower(frPower);
        Bot.rlMotor.setPower(rlPower);
        Bot.rrMotor.setPower(rrPower);

    }

    public void drive(){

        leftStickYVal = -gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        //Field Centric
        flSpeed = leftStickYVal+leftStickXVal+rightStickXVal;
        flSpeed = Range.clip(flSpeed, -1, 1);
        frSpeed = leftStickYVal-leftStickXVal-rightStickXVal;
        frSpeed = Range.clip(frSpeed, -1, 1);
        rlSpeed = leftStickYVal-leftStickXVal+rightStickXVal;
        rlSpeed = Range.clip(rlSpeed, -1, 1);
        rrSpeed = leftStickYVal+leftStickXVal-rightStickXVal;
        rrSpeed = Range.clip(rrSpeed, -1, 1);

        //Robot Centric - Andrea prefers
//        flSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
//        flSpeed = Range.clip(flSpeed, -1, 1);
//        frSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
//        frSpeed = Range.clip(frSpeed, -1, 1);
//        rlSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
//        rlSpeed = Range.clip(rlSpeed, -1, 1);
//        rrSpeed = leftStickYVal + rightStickXVal - leftStickXVal;
//        rrSpeed = Range.clip(rrSpeed, -1, 1);

        if (flSpeed <= powerThreshold && flSpeed >= -powerThreshold){
            flSpeed = 0;
            Bot.flMotor.setPower(flSpeed);
        }else{
            Bot.flMotor.setPower(flSpeed * speedMultiply);
        }

        if(frSpeed <= powerThreshold && frSpeed >= -powerThreshold){
            frSpeed = 0;
            Bot.frMotor.setPower(frSpeed);
        }else{
            Bot.frMotor.setPower(frSpeed * speedMultiply);
        }

        if(rlSpeed <= powerThreshold && rlSpeed >= -powerThreshold) {
            rlSpeed = 0;
            Bot.rlMotor.setPower(rlSpeed);
        }else{
            Bot.rlMotor.setPower(rlSpeed * speedMultiply);
        }

        if(rrSpeed <= powerThreshold && rrSpeed >= -powerThreshold) {
            rrSpeed = 0;
            Bot.rrMotor.setPower(rrSpeed);
        }else{
            Bot.rrMotor.setPower(rrSpeed * speedMultiply);
        }
    }

    public void intakeControl(){
        if(gamepad2.y){
            Bot.ballOuttake();
        }
        if(gamepad2.x){
            Bot.ballIntake();
        }
    }

    public void speedControl(){
        if(gamepad1.dpad_up){
            speedMultiply = 1;
        }
        else if(gamepad1.dpad_down){
            speedMultiply = .5;
        }

    }

    public void telemetryOutput(){
        telemetry.update();
    }
}
