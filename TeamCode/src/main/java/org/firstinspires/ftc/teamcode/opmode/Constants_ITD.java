package org.firstinspires.ftc.teamcode.opmode;

public class Constants_ITD {

    // Extendo
    public static final double extendoPower = 0.4;
    public static final int extendoRetracted = 0;
    public static final int extendoTransferAmount = 95;
    public static final int extendoTransfer = 75 + extendoTransferAmount;
    public static final int extendoActive = 200; //200
    public static final int extendoIncrement = 50;
    public static final int extendoMax = 455;

    // Intake Grabber
    public static final double intakeGrabberNeutral = 0.24; //0.24 roller
    public static final double intakeGrabberOpen = 0.32; //0.19 roller
    public static final double intakeGrabberClosed = 0.18; //0.28 roller

    // Intake Pronator
    public static final double intakePronatorNeutral = 0.492;
    public static final double intakePronatorLeft = 0;
    public static final double intakePronatorRight = 1;

    // Intake Flexor
    public static final double intakeFlexorRetracted = 0.07;
    public static final double intakeFlexorTransfer = 0.06 + 0.18; //0.1 + 0.18 lower grabber
    public static final double intakeFlexorHover = 0.77 + 0.18; //0.71 + 0.18 lower grabber
    public static final double intakeFlexorGrab = 0.76 + 0.18; //0.807 + 0.18 lower grabber
    public static final double intakeFlexorVertical = 0.45;

    // Intake Elbow
    public static final double intakeElbowRetracted = 0.3;
    public static final double intakeElbowTransfer = 0.54; //0.58 lower grabber
    public static final double intakeElbowHover = 0.63; //0.59 lower grabber
    public static final double intakeElbowGrab = 0.68; //0.64 lower grabber

    // Elevator
    public static final double elevatorPower = 0.7;
    public static final int elevatorTransferDown = 600;
    public static final int elevatorRetracted = 0;
    public static final int elevatorTransfer = 390;
    public static final int elevatorSpecimen = 730;
    public static final int elevatorLowChamber = 730;
    public static final int elevatorHighChamber = 1015; //1860

    // amount of ticks to move down to score on chambers
    public static final int elevatorScoring = 100; // 160
    public static final int elevatorBasket = 2650; //2800 - 3100

    // Timers
    public static final double transferTime1 = 0.5; // seconds //0.7
    public static final double transferTime2 = 0.3 + transferTime1; // seconds
    public static final double transferTime3 = 0.1 + transferTime2; //0.5
    public static final double swingOutTime = 0.2;
    public static final double pluckTime1 = 0.1;
    public static final double pluckTime2 = 0.3 + pluckTime1;
    public static final double pluckTime3 = 0.3 + pluckTime2;
    public static final double specimenScoringTime = 0.3;
    public static final double passThroughTime = 0.4;

    // Outtake Elbow
    public static final double outtakeElbowTransfer = 0;
    public static final double outtakeElbowBasket = 0.87;
    public static final double outtakeElbowAboveChamber = 0.48;
    public static final double outtakeElbowBelowChamber = 0.4;
    // TODO: GET THIS VALUE
    public static final double outtakeElbowSpecimen = 0.32;
    public static final double outtakeElbowHorizontal = 0.563;
    public static final double outtakeElbowInitialize = 0.38;

    // Outtake Pronator
    public static final double outtakePronatorTransfer = 0.685;
    public static final double outtakePronatorChamber = 0.685;
    public static final double outtakePronatorBasket = 0.125;
    public static final double outtakePronatorRig = 0.685;

    // Outtake Flexor
    public static final double outtakeFlexorTransfer = 0.305;
    public static final double outtakeFlexorAboveChamber = 0.35;
    public static final double outtakeFlexorBelowChamber = 0.35; //0.35
    public static final double outtakeFlexorBasket = 0.41;
    public static final double outtakeFlexorRig = 0;
    public static final double outtakeFlexorStraight = 0.62;
    public static final double outtakeFlexorSpecimen = 0.525;
    public static final double outtakeFlexorInitialize = 0.47;

    // Outtake Grabber
    public static final double outtakeGrabberLeftOpen = 0.78; //0.83
    public static final double outtakeGrabberRightOpen = 0.78;
    public static final double outtakeGrabberLeftClose = 0.96;
    public static final double outtakeGrabberRightClose = 0.96;

    // Lever
    public static final double leverPower = 1;
    public static final int leverRetracted = 0;
    public static final int leverActive = 0;
}
