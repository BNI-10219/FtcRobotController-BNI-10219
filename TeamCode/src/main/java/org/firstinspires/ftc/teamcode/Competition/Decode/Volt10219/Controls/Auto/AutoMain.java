package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;

public abstract class AutoMain extends LinearOpMode {
    public DecodeBot Bot = new DecodeBot();

    private Limelight3A limelight;

//    limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        limelight.pipelineSwitch(0);

   // limelight.start();

    public void camera(){
        LLResult result = limelight.getLatestResult();
        telemetry.addData("LL Result: ", result);
        //Fiducial results is a list - if the list size(number of AT seen) is 1 - show what tag it is
        if (result.getFiducialResults().size() == 1){
            telemetry.addData("Tags", result.getFiducialResults().get(0).getFiducialId());
        }
        telemetry.update();

    }
}
