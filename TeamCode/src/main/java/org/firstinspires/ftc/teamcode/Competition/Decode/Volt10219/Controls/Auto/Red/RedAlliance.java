package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.AutoMain;

public abstract class RedAlliance extends AutoMain {

    protected static final int PPG_TAG_ID = 23;
    protected static final int PGP_TAG_ID = 22;
    protected static final int GPP_TAG_ID = 21;

    // GoBilda Pinpoint-based Creep Control
    protected Pose creepStartPose;
    protected double creepTargetY;           // field Y we want to hold
    protected double creepTargetHeading;     // heading we want to hold

    protected double creepForwardPower = 0.50;   // slow forward power (tune)
    protected double creepLatKp = 0.03;          // lateral (Y) correction gain (tune)
    protected double creepHeadingKp = 0.05;      // heading correction gain (tune)

    protected double creepTargetDistanceIn = 26.5;  // how far to creep (inches)
    protected double creepTimeoutS = 4.0;

    // For Testing without Motif
    protected final Pose intake = new Pose(96, 34, Math.toRadians(0));
    protected final Pose intakePickupEnd = new Pose(112, 34, Math.toRadians(0));

    // Preparing for Intake Using Motifs... Not Used Yet.
    protected final Pose PPGPose = new Pose(96, 81, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    protected final Pose PPGPosePickup = new Pose(112, 81, Math.toRadians(0));

    protected final Pose PGPPose = new Pose(96, 57.5, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    protected final Pose PGPPosePickup = new Pose(112, 57.5, Math.toRadians(0));

    protected final Pose GPPPose = new Pose(96, 34, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    protected final Pose GPPPosePickup = new Pose(112, 34, Math.toRadians(0));


}
