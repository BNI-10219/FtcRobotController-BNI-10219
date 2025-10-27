package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ProgramPedro;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;


public abstract class AutoMainProgram extends OpMode {
    public DecodeBot Bot = new DecodeBot();

    private Limelight3A limelight;

    private boolean autoPosition = false;

    double targetTX = 23;
    double targetTA = 3;
    double llTolerance = 1.25;

    public void autoStartUp(){
        Bot.initRobot(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Awaiting Start");
        telemetry.update();
    }

    public void camera(){
        LLResult result = limelight.getLatestResult();
        telemetry.addData("LL Result: ", result);
        //Fiducial results is a list - if the list size(number of AT seen) is 1 - show what tag it is
        if (result.getFiducialResults().size() == 1){
            telemetry.addData("Tags", result.getFiducialResults().get(0).getFiducialId());
        }
        telemetry.update();

    }



    public void autoPositioning(){
        autoPosition = false;
        if (!autoPosition){
            return;
        }

        LLResult result = limelight.getLatestResult();
        double txDifference = result.getTx() - targetTX;
        double taDifference = result.getTy() - targetTA;


        if (autoPosition){
            if (txDifference < 0 - llTolerance){
                Bot.strafeRight(1);

            }
            else if (txDifference > 0 + llTolerance){
                Bot.strafeLeft(1);
            }
            else if((txDifference> 0 - llTolerance) && (txDifference < 0 + llTolerance)){
                Bot.stopMotors();
            }

            if (taDifference > 0 + llTolerance){
                Bot.driveForward(0.5);
            }
            else if (taDifference < 0 + llTolerance){
                Bot.stopMotors();
            }

        }

        telemetry.addData("Tx Difference: ", txDifference);
        telemetry.addData("Ta Difference: ", taDifference);
        telemetry.addData("LL Ta:", result.getTa() );
        telemetry.addData("LL Tx: ", result.getTx());
        telemetry.update();

    }

    //autoPositioning(); - call in whichever state I want it to be in
    public void loop(){

    }


}
