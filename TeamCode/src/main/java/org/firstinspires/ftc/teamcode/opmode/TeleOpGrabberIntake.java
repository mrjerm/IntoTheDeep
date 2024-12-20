package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorBasket;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorHighChamber;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorLowChamber;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorPower;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorScoring;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorSpecimen;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorTransfer;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoActive;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoIncrement;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoMax;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoPower;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoRetracted;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoTransfer;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoTransferAmount;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeElbowGrab;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeElbowHover;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeElbowRetracted;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeElbowTransfer;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeFlexorGrab;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeFlexorHover;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeFlexorRetracted;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeFlexorTransfer;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeFlexorVertical;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeGrabberClosed;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeGrabberOpen;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakePronatorLeft;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakePronatorNeutral;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakePronatorRight;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeElbowAboveChamber;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeElbowBasket;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeElbowBelowChamber;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeElbowHorizontal;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeElbowInitialize;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeElbowSpecimen;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeElbowTransfer;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeFlexorAboveChamber;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeFlexorBasket;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeFlexorBelowChamber;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeFlexorInitialize;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeFlexorSpecimen;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeFlexorStraight;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeFlexorTransfer;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeGrabberLeftClose;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeGrabberLeftOpen;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeGrabberRightClose;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeGrabberRightOpen;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakePronatorBasket;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakePronatorChamber;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakePronatorTransfer;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.passThroughTime;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.pluckTime1;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.pluckTime2;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.pluckTime3;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.specimenScoringTime;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.swingOutTime;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.transferTime1;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.transferTime2;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.transferTime3;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Single Player")
public class TeleOpGrabberIntake extends OpMode {

    List<LynxModule> allHubs;

    // Declare motors
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx extendo;
    private DcMotorEx elevatorLeft, elevatorRight;

    // Declare servos
    private Servo intakeGrabber, intakePronator, intakeFlexor, intakeElbow;
    private Servo outtakePronator, outtakeFlexor, outtakeElbowLeft, outtakeElbowRight, outtakeGrabberLeft, outtakeGrabberRight;

    // Extendo SM
    private enum ExtendoState {
        RETRACTED, TRANSFER, ACTIVE
    }
    private ExtendoState extendoState = ExtendoState.RETRACTED;
    // Elevator SM
    private enum ElevatorState {
        RETRACTED, TRANSFER, LOWCHAMBER, HIGHCHAMBER, BASKET, SPECIMEN
    }
    private ElevatorState elevatorState = ElevatorState.LOWCHAMBER;

    // Robot Mode SM
    private enum RobotState{
        SPECIMEN, SAMPLE, HANG
    }
    private RobotState robotState = RobotState.SAMPLE;

    private boolean extendoForwardPrev = false;
    private boolean extendoBackPrev = false;
    private boolean extendoRetractPrev = false;
    private boolean leftAndRightPrev = false;
    private boolean upPrev = false;
    private boolean downPrev = false;
    private boolean touchpadPrev = false;
    private boolean rightStickButtonPrev = false;

    // timer BS
    private boolean transferAvailable = true;
    private boolean swingOutAvailable = true;
    private boolean pluckAvailable = true;
    private boolean specimenScoringAvailable = true;
    private boolean passThroughAvailable = true;
    private double transferTimeStamp = 0;
    private double swingOutTimeStamp = 0;
    private double pluckTimeStamp = 0;
    private double specimenScoringTimeStamp = 0;
    private double passThroughTimer = 0;

    private int extendoPosition = 0;
    private double speedLimit = 1; //scales the speed of the drivetrain
    private double turnPower = 0.6; //scales the turning speed, this happens before the speed limit is applied

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        // Initialize the motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "Motor FL");
        frontRight = hardwareMap.get(DcMotorEx.class, "Motor FR");
        backLeft = hardwareMap.get(DcMotorEx.class, "Motor BL");
        backRight = hardwareMap.get(DcMotorEx.class, "Motor BR");
        extendo = hardwareMap.get(DcMotorEx.class, "Motor Extendo");
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "Motor Elevator L");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "Motor Elevator R");

        // Reverse the direction of specific motors as needed for proper mecanum behavior
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motors
//        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize Servos
        intakeGrabber = hardwareMap.get(Servo.class, "Servo Intake Grabber");
        intakePronator = hardwareMap.get(Servo.class, "Servo Intake Pronator");
        intakeFlexor = hardwareMap.get(Servo.class, "Servo Intake Flexor"); //v4b right
        intakeElbow = hardwareMap.get(Servo.class, "Servo Intake Elbow"); //v4b left

        outtakePronator = hardwareMap.get(Servo.class, "Servo Outtake Pronator");
        outtakeFlexor = hardwareMap.get(Servo.class, "Servo Outtake Flexor");
        outtakeElbowLeft = hardwareMap.get(Servo.class, "Servo Outtake Elbow L");
        outtakeElbowRight = hardwareMap.get(Servo.class, "Servo Outtake Elbow R");
        outtakeGrabberLeft = hardwareMap.get(Servo.class, "Servo Outtake Grabber L");
        outtakeGrabberRight = hardwareMap.get(Servo.class, "Servo Outtake Grabber R");

        outtakeElbowLeft.setDirection(Servo.Direction.REVERSE);
        outtakeElbowRight.setDirection(Servo.Direction.REVERSE);

        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


    }

    @Override
    public void start() {
        resetRuntime();
        setIntake(intakePronatorNeutral, intakeFlexorRetracted, intakeElbowHover);
        setElevator(elevatorLowChamber);
        setOuttakeArm(outtakeElbowInitialize, outtakeFlexorInitialize, outtakePronatorChamber);
    }

    @Override
    public void loop() {
        controlSpeed();
        drive();
        setMode(gamepad1.right_stick_button && !rightStickButtonPrev);
        switch (robotState){
            case SAMPLE:
                controlExtendoSample(gamepad1.y, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.a);
                controlIntakeSample(gamepad1.left_trigger > 0.1, gamepad1.right_trigger > 0.1, gamepad1.x, gamepad1.b, gamepad1.left_stick_button, gamepad1.touchpad);
                controlOuttakeSample(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.touchpad && !touchpadPrev, gamepad1.left_trigger > 0.1, gamepad1.right_trigger > 0.1, gamepad1.a);
                break;
            case SPECIMEN:
                if (!swingOutAvailable && getRuntime() - swingOutTimeStamp >= swingOutTime && getRuntime() - swingOutTimeStamp < swingOutTime + 0.2) {
                    setOuttakeArm(outtakeElbowSpecimen, outtakeFlexorSpecimen, outtakePronatorChamber);
                    setElevator(elevatorSpecimen);
                    swingOutAvailable = true;
                }
                controlOuttakeSpecimen(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.y, gamepad1.a, gamepad1.left_trigger > 0.5, gamepad1.right_trigger > 0.1, gamepad1.touchpad);

        }

//        telemetry.addLine("outtake state: " + elevatorState);
//        telemetry.addLine("speed limit: " + speedLimit);
//        telemetry.update();
        updateDebounce();
    }

    private void controlSpeed(){
        if(elevatorLeft.getCurrentPosition() > elevatorTransfer + 30){
            turnPower = 0.6;
        } else{
            turnPower = 0.4;
        }
        if (elevatorState == ElevatorState.SPECIMEN){
            speedLimit = 0.6;
        }
        if (elevatorState == ElevatorState.HIGHCHAMBER){
            speedLimit = 0.8;
        }
        if (elevatorState == ElevatorState.TRANSFER){
            speedLimit = 0.8;
        }
        if (elevatorState == ElevatorState.BASKET){
            speedLimit = 0.5;
        }
        if (elevatorState == ElevatorState.LOWCHAMBER){
            speedLimit = 1;
        }

    }

    private void controlOuttakeSpecimen(boolean up, boolean down, boolean active, boolean inactive, boolean open, boolean close, boolean score){
        switch (elevatorState){
            case SPECIMEN:
                if (active) {
                    elevatorState = ElevatorState.LOWCHAMBER;
                    setElevator(elevatorLowChamber);
                    setOuttakeArm(outtakeElbowAboveChamber, outtakeFlexorAboveChamber, outtakePronatorChamber);
                    specimenScoringAvailable = true;
                }
                if (open) {
                    setOuttakeGrabber(true);
                }
                if (close){
                    setOuttakeGrabber(false);
                }
                break;
            case LOWCHAMBER:
                if (up) {
                    elevatorState = ElevatorState.HIGHCHAMBER;
                    setElevator(elevatorHighChamber);
//                    setOuttakeArm(outtakeElbowAboveChamber, outtakeFlexorAboveChamber, outtakePronatorChamber);
                    setOuttakeArm(outtakeElbowHorizontal, outtakeFlexorStraight, outtakePronatorChamber);
                }
                if (inactive) {
                    elevatorState = ElevatorState.SPECIMEN;
                    setElevator(elevatorSpecimen);
                    setOuttakeArm(outtakeElbowSpecimen, outtakeFlexorSpecimen, outtakePronatorChamber);
                    setOuttakeGrabber(true);
                }
                if (open) {
                    setOuttakeGrabber(true);
                }
                if (close){
                    setOuttakeGrabber(false);
                }
                if (score && specimenScoringAvailable) {
                    specimenScoringAvailable = false;
                    specimenScoringTimeStamp = getRuntime();
                    setElevator(elevatorLowChamber - elevatorScoring);
                    setOuttakeArm(outtakeElbowBelowChamber, outtakeFlexorBelowChamber, outtakePronatorChamber);
                }
                if (!specimenScoringAvailable && getRuntime() - specimenScoringTimeStamp >= specimenScoringTime && getRuntime() - specimenScoringTimeStamp < specimenScoringTime + 0.3){
                    setOuttakeGrabber(true);
                    specimenScoringAvailable = true;
                }
                break;
            case HIGHCHAMBER:
                if (down) {
                    elevatorState = ElevatorState.LOWCHAMBER;
                    setElevator(elevatorLowChamber);
                    setOuttakeArm(outtakeElbowAboveChamber, outtakeFlexorAboveChamber, outtakePronatorChamber);
                }
                if (inactive) {
                    elevatorState = ElevatorState.SPECIMEN;
                    setElevator(elevatorSpecimen);
                    setOuttakeArm(outtakeElbowSpecimen, outtakeFlexorSpecimen, outtakePronatorChamber);
                    setOuttakeGrabber(true);
                }
                if (open) {
                    setOuttakeGrabber(true);
                }
                if (close){
                    setOuttakeGrabber(false);
                }
//                if (score && specimenScoringAvailable) {
//                    specimenScoringAvailable = false;
//                    specimenScoringTimeStamp = getRuntime();
//                    setElevator(elevatorHighChamber - elevatorScoring);
//                    setOuttakeArm(outtakeElbowBelowChamber, outtakeFlexorBelowChamber, outtakePronatorChamber);
//                }
//                if (!specimenScoringAvailable && getRuntime() - specimenScoringTimeStamp >= specimenScoringTime && getRuntime() - specimenScoringTimeStamp < specimenScoringTime + 0.3){
//                    setOuttakeGrabber(true);
//                    specimenScoringAvailable = true;
//                }
                break;
        }
    }


    private void setMode(boolean toggle){
        if (toggle){
            switch (robotState){
                case SAMPLE:
                    robotState = RobotState.SPECIMEN;
                    elevatorState = ElevatorState.SPECIMEN;

                    swingOutAvailable = false;
                    swingOutTimeStamp = getRuntime();

                    setElevator(elevatorSpecimen);
                    setIntake(intakePronatorNeutral, intakeFlexorVertical, intakeElbowHover);
                    setExtendo(extendoRetracted);
                    setOuttakeGrabber(true);
                    break;
                case SPECIMEN:
                    robotState = RobotState.SAMPLE;
                    elevatorState = ElevatorState.RETRACTED;
                    extendoState = ExtendoState.RETRACTED;

                    setElevator(elevatorTransfer);
                    setOuttakeArm(outtakeElbowTransfer, outtakeFlexorTransfer, outtakePronatorTransfer);
                    setOuttakeGrabber(true);

                    extendoPosition = extendoRetracted;
                    setIntake(intakePronatorNeutral, intakeFlexorVertical, intakeElbowHover);
                    setElevator(elevatorLowChamber);
                    setOuttakeArm(outtakeElbowHorizontal, outtakeFlexorTransfer, outtakePronatorTransfer);
                    break;
            }
        }
    }

    private void updateDebounce(){
        touchpadPrev = gamepad1.touchpad;
        rightStickButtonPrev = gamepad1.right_stick_button;
    }

    private void drive() {
        // Get joystick values
        float x1 = gamepad1.left_stick_x;   // Strafe (left/right)
        float y1 = -gamepad1.left_stick_y;  // Forward/backward (inverted Y-axis)
        float x2 = (float) (gamepad1.right_stick_x * turnPower);  // Rotation (clockwise/counterclockwise)

        // Calculate power for each motor
        double frontLeftPower = y1 + x1 + x2;
        double frontRightPower = y1 - x1 - x2;
        double backLeftPower = y1 - x1 + x2;
        double backRightPower = y1 + x1 - x2;

        // Normalize the powers if any value exceeds 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Apply the calculated power to the motors
        frontLeft.setPower(frontLeftPower * speedLimit);
        frontRight.setPower(frontRightPower * speedLimit);
        backLeft.setPower(backLeftPower * speedLimit);
        backRight.setPower(backRightPower * speedLimit);
    }

    private void setIntake(double pronation, double flexion, double elbow){
        intakePronator.setPosition(pronation);
        intakeFlexor.setPosition(flexion);
        intakeElbow.setPosition(elbow);
    }

    private void controlExtendoSample(boolean activate, boolean forward, boolean back, boolean retract){
        switch (extendoState){
            case ACTIVE:
                // forward and back for extendo
                if (forward && !extendoForwardPrev){
                    extendoPosition+=extendoIncrement;
                    extendoPosition = Range.clip(extendoPosition, extendoActive, extendoMax);
                }

                if (back && !extendoBackPrev) {
                    extendoPosition-=extendoIncrement;
                    extendoPosition = Range.clip(extendoPosition, extendoActive, extendoMax);
                }

                //TODO: Color sensor logic
                if (retract && !extendoRetractPrev) {
                    extendoState = ExtendoState.TRANSFER;
                    setIntake(intakePronatorNeutral, intakeFlexorTransfer, intakeElbowTransfer);
                    extendoPosition = extendoTransfer;
                    intakePronator.setPosition(intakePronatorNeutral); // avoid destroying robot
                }
                break;
            case TRANSFER:
                if (retract && !extendoRetractPrev) {
                    extendoState = ExtendoState.RETRACTED;
                    extendoPosition = extendoRetracted;
                    setIntake(intakePronatorNeutral, intakeFlexorRetracted, intakeElbowHover);
                    elevatorState = ElevatorState.LOWCHAMBER;
                    setElevator(elevatorLowChamber);
                    setOuttakeArm(outtakeElbowTransfer, outtakeFlexorTransfer, outtakePronatorTransfer);
                }
                if (activate) {
                    extendoState = ExtendoState.ACTIVE;
                    setIntake(intakePronatorNeutral, intakeFlexorHover, intakeElbowHover);
                    extendoPosition = extendoActive + extendoIncrement * 2;
                    intakeGrabber.setPosition(intakeGrabberOpen);
                }
            case RETRACTED:
                if (activate) {
                    extendoState = ExtendoState.ACTIVE;
                    setIntake(intakePronatorNeutral, intakeFlexorHover, intakeElbowHover);
                    extendoPosition = extendoActive + extendoIncrement * 2;
                    intakeGrabber.setPosition(intakeGrabberOpen);
                }
                break;
        }
        extendoRetractPrev = retract;
        extendoBackPrev = back;
        extendoForwardPrev = forward;

        setExtendo(extendoPosition);
    }

    private void controlIntakeSample(boolean open, boolean close, boolean left, boolean right, boolean passthrough, boolean pluck){

        if (pluck && extendoState == ExtendoState.ACTIVE && pluckAvailable) {
            pluckAvailable = false;
            pluckTimeStamp = getRuntime();
            intakeElbow.setPosition(intakeElbowGrab);
            intakeFlexor.setPosition(intakeFlexorGrab);
        }
        if (getRuntime() - pluckTimeStamp >= pluckTime1 && getRuntime() - pluckTimeStamp < pluckTime2) {
            intakeGrabber.setPosition(intakeGrabberClosed);
        }
        if (getRuntime() - pluckTimeStamp >= pluckTime2 && getRuntime() - pluckTimeStamp < pluckTime2 * 2) {
            extendoState = ExtendoState.TRANSFER;
            setIntake(intakePronatorNeutral, intakeFlexorTransfer, intakeElbowTransfer);
            extendoPosition = extendoTransfer;
            intakePronator.setPosition(intakePronatorNeutral);
            setOuttakeGrabber(true);
            elevatorState = ElevatorState.TRANSFER;
            setElevator(elevatorTransfer);
            setOuttakeArm(outtakeElbowTransfer, outtakeFlexorTransfer, outtakePronatorTransfer);
            setOuttakeGrabber(true);

            pluckAvailable = true;
        }
        if (open) {
            intakeGrabber.setPosition(intakeGrabberOpen);
        } else if (close) {
            intakeGrabber.setPosition(intakeGrabberClosed);
        }

        if (left && right){
            intakePronator.setPosition(intakePronatorNeutral);
        } else if (left && !leftAndRightPrev){
            intakePronator.setPosition(Range.clip(intakePronator.getPosition() - 0.1, intakePronatorLeft, intakePronatorRight));
        } else if (right && !leftAndRightPrev) {
            intakePronator.setPosition(Range.clip(intakePronator.getPosition() + 0.1, intakePronatorLeft, intakePronatorRight));
        }
        leftAndRightPrev = left || right;

        if (passthrough && extendoState == ExtendoState.RETRACTED) {
            setIntake(intakePronatorNeutral, intakeFlexorRetracted, intakeElbowRetracted);
            passThroughAvailable = false;
            passThroughTimer = getRuntime();
        }
        if (!passThroughAvailable && getRuntime() - passThroughTimer > passThroughTime) {
            passThroughAvailable = true;
            intakeGrabber.setPosition(intakeGrabberOpen);
        }
    }

    // TODO: outtake pronator
    private void controlOuttakeSample(boolean up, boolean down, boolean transfer, boolean open, boolean close, boolean goToTransfer){
        if (goToTransfer && extendoState != ExtendoState.RETRACTED) {
            elevatorState = ElevatorState.RETRACTED;
            extendoState = ExtendoState.TRANSFER;
        }
        switch (elevatorState){
            case RETRACTED:
                elevatorState = ElevatorState.TRANSFER;
                setElevator(elevatorTransfer);
                setOuttakeArm(outtakeElbowTransfer, outtakeFlexorTransfer, outtakePronatorTransfer);
                setOuttakeGrabber(true);
                break;
            case TRANSFER:
                if (transferAvailable && up && !upPrev){
                    swingOutTimeStamp = getRuntime();
                    elevatorState = ElevatorState.LOWCHAMBER;
                    setElevator(elevatorLowChamber);
                }
                if (transfer && transferAvailable && extendoState == ExtendoState.TRANSFER && getRuntime() - pluckTimeStamp > pluckTime3) {
                    transferAvailable = false;
                    transferTimeStamp = getRuntime();
                    extendoPosition = extendoTransfer - extendoTransferAmount - (getRuntime() > 60 ? 10 : 0);
                    setOuttakeGrabber(true);
                }
                if (getRuntime() - transferTimeStamp >= transferTime1 && getRuntime() - transferTimeStamp < transferTime1 + 0.1){
                    setOuttakeGrabber(false);
                }
                if (getRuntime() - transferTimeStamp >= transferTime2 && getRuntime() - transferTimeStamp < transferTime2 + 0.1){
                    intakeGrabber.setPosition(intakeGrabberOpen);
                }
                if (getRuntime() - transferTimeStamp >= transferTime3 && getRuntime() - transferTimeStamp < transferTime3 + 0.1){
                    transferAvailable = true;
                    extendoPosition = extendoTransfer;
                }
                break;
            case LOWCHAMBER:
                if (getRuntime() - swingOutTimeStamp >= swingOutTime && getRuntime() - swingOutTimeStamp < swingOutTime + 0.1){
                    setOuttakeArm(outtakeElbowAboveChamber, outtakeFlexorAboveChamber, outtakePronatorChamber);
                }
                if (up && !upPrev) {
                    elevatorState = ElevatorState.HIGHCHAMBER;
                    setElevator(elevatorHighChamber);
                    setOuttakeArm(outtakeElbowAboveChamber, outtakeFlexorAboveChamber, outtakePronatorChamber);
                }
                if (down && !downPrev) {
                    elevatorState = ElevatorState.RETRACTED;
                }
                if (open) {
                    setOuttakeGrabber(true);
                } else if (close) {
                    setOuttakeGrabber(false);
                }
                break;
            case HIGHCHAMBER:
                if (up && !upPrev) {
                    elevatorState = ElevatorState.BASKET;
                    setElevator(elevatorBasket);
                    setOuttakeArm(outtakeElbowBasket, outtakeFlexorBasket, outtakePronatorBasket);
                }
                if (down && !downPrev){
                    elevatorState = ElevatorState.LOWCHAMBER;
                    setElevator(elevatorLowChamber);
                    setOuttakeArm(outtakeElbowAboveChamber, outtakeFlexorAboveChamber, outtakePronatorChamber);
                }
                if (open) {
                    setOuttakeGrabber(true);
                } else if (close) {
                    setOuttakeGrabber(false);
                }
                break;
            case BASKET:
                if (down && !downPrev) {
                    elevatorState = ElevatorState.HIGHCHAMBER;
                    setOuttakeArm(outtakeElbowAboveChamber, outtakeFlexorAboveChamber, outtakePronatorChamber);
                    setElevator(elevatorHighChamber);
                }
                if (open) {
                    setOuttakeGrabber(true);
                } else if (close) {
                    setOuttakeGrabber(false);
                }
                break;
        }
        upPrev = up;
        downPrev = down;
    }

    private void setOuttakeArm(double elbow, double flexion, double pronation){
        outtakeElbowLeft.setPosition(elbow);
        outtakeElbowRight.setPosition(elbow);
        outtakeFlexor.setPosition(flexion);
        outtakePronator.setPosition(pronation);
    }

    private void setOuttakeGrabber(boolean open){
        if (open) {
            outtakeGrabberLeft.setPosition(outtakeGrabberLeftOpen);
            outtakeGrabberRight.setPosition(outtakeGrabberRightOpen);
        } else {
            outtakeGrabberLeft.setPosition(outtakeGrabberLeftClose);
            outtakeGrabberRight.setPosition(outtakeGrabberRightClose);
        }
    }

    private void setExtendo(int position){
        extendo.setTargetPosition(position);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(extendoPower);
    }

    private void setElevator(int position){
        elevatorLeft.setTargetPosition(position);
        elevatorRight.setTargetPosition(position);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(elevatorPower);
        elevatorRight.setPower(elevatorPower);
    }

}
