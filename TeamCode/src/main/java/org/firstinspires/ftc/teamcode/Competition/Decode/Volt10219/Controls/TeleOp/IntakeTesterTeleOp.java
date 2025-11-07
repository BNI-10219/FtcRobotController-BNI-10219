package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;

@Disabled
@TeleOp(name = "Intake Tester TeleOp")
public class IntakeTesterTeleOp extends OpMode {
    double leftStickYVal;
    double leftStickXVal;
    double rightStickYVal;
    double rightStickXVal;

    double powerThreshold = 0;
    double speedMultiply = 1;


    double targetTX = 23;
    double targetTA = 3;
    double llTolerance = 1.25;


    double flSpeed;
    double frSpeed;
    double rlSpeed;
    double rrSpeed;

    public DecodeBot Bot = new DecodeBot();

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
        intakeControl();
        telemetryOutput();
    }


    public void intakeControl(){
        if(gamepad2.dpad_right) {
            Bot.ballIntakeOne.setPower(-1);
            //Bot.ballIntakeTwo.setPower(-1);
        }
        if(gamepad2.dpad_left){
            Bot.ballIntakeOne.setPower(1);
            //Bot.ballIntakeTwo.setPower(1);
        }
        if(gamepad2.b){
            Bot.ballIntakeOne.setPower(0);
            Bot.ballIntakeTwo.setPower(0);
        }
    }


    public void telemetryOutput(){
        telemetry.addData("Launcher One: ", Math.abs(Bot.ballLaunchOne.getVelocity()));
        telemetry.addData("Launcher Two: ", Bot.ballLaunchTwo.getVelocity());
        telemetry.update();
    }
}
