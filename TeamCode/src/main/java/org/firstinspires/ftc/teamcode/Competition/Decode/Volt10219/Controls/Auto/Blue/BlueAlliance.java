package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.AutoMain;

public abstract class BlueAlliance extends AutoMain {

    protected static final int PPG_TAG_ID = 23;
    protected static final int PGP_TAG_ID = 22;
    protected static final int GPP_TAG_ID = 21;

    // GoBilda Pinpoint-based Creep Control
    protected Pose creepStartPose;
    protected double creepTargetY;           // field Y we want to hold
    protected double creepTargetHeading;     // heading we want to hold

    protected double creepForwardPower = 0.50;   // slow forward power (tune)
    protected double creepLatKp = 0.02;          // lateral (Y) correction gain (tune)
    protected double creepHeadingKp = 0.04;      // heading correction gain (tune)

    protected double creepTargetDistanceIn = 26.5;  // how far to creep (inches)
    protected double creepTimeoutS = 5.0;

    // For Testing without Motif
    protected final Pose intake = new Pose(48, 34, Math.toRadians(180));
    protected final Pose intakePickupEnd = new Pose(32, 34, Math.toRadians(180));

    // Preparing for Intake Using Motifs... Not Used Yet.



}
