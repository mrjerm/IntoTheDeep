package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * TODO: raise the elevator fro transfer
 * touchpad button for swinging outtake arm out because outtake arm obliterates the robot when swinging out at same time as elevator raising
 * extendo doesnt retract when retracted
 * intake grabber to neutral when activating
 * incorporate elevator drop when scoring specimen
 * test basket height
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpRollerIntake extends OpMode {
    // Declare motors
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx extendo;
    private DcMotorEx elevatorLeft, elevatorRight;

    // Declare servos
    private CRServo intakeLeft, intakeRight;
    private Servo intakeGrabber, intakePronator, intakeFlexor, intakeElbow;
    private Servo outtakePronator, outtakeFlexor, outtakeElbowLeft, outtakeElbowRight, outtakeGrabberLeft, outtakeGrabberRight;

    // Extendo SM
    private enum ExtendoState {
        RETRACTED, TRANSFER, ACTIVE
    }
    private ExtendoState extendoState = ExtendoState.RETRACTED;

    private enum IntakeState{
        OFF, ON
    }
    private IntakeState intakeState = IntakeState.OFF;

    // Elevator SM
    private enum ElevatorState {
        RETRACTED, TRANSFER, LOWCHAMBER, HIGHCHAMBER, BASKET
    }
    private ElevatorState elevatorState = ElevatorState.LOWCHAMBER;

    // Outtake arm SM
    private enum OuttakeArmState {
        TRANSFER, CHAMBER, BUCKET, RIGOUT, RIGIN
    }
    private OuttakeArmState outtakeArmState = OuttakeArmState.TRANSFER;

    private boolean extendoForwardPrev = false;
    private boolean extendoBackPrev = false;
    private boolean extendoRetractPrev = false;
    private boolean leftAndRightPrev = false;
    private boolean upPrev = false;
    private boolean downPrev = false;

    private boolean transferAvailable = true;
    private boolean swingOutAvailable = true;
    private double transferTimeStamp = 0;
    private double swingOutTimeStamp = 0;

    private int extendoPosition = 0;
    private double speedLimit = 0.7;

    @Override
    public void init() {
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
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize Servos
        intakeLeft = hardwareMap.get(CRServo.class, "Servo Intake L");
        intakeRight = hardwareMap.get(CRServo.class, "Servo Intake R");
        intakeGrabber = hardwareMap.get(Servo.class, "Servo Intake Grabber");
        intakePronator = hardwareMap.get(Servo.class, "Servo Intake Pronator");
        intakeFlexor = hardwareMap.get(Servo.class, "Servo Intake Flexor"); //v4b right
        intakeElbow = hardwareMap.get(Servo.class, "Servo Intake Elbow"); //v4b left

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakePronator = hardwareMap.get(Servo.class, "Servo Outtake Pronator");
        outtakeFlexor = hardwareMap.get(Servo.class, "Servo Outtake Flexor");
        outtakeElbowLeft = hardwareMap.get(Servo.class, "Servo Outtake Elbow L");
        outtakeElbowRight = hardwareMap.get(Servo.class, "Servo Outtake Elbow R");
        outtakeGrabberLeft = hardwareMap.get(Servo.class, "Servo Outtake Grabber L");
        outtakeGrabberRight = hardwareMap.get(Servo.class, "Servo Outtake Grabber R");

        outtakeElbowLeft.setDirection(Servo.Direction.REVERSE);
        outtakeElbowRight.setDirection(Servo.Direction.REVERSE);


        setIntake(intakePronatorNeutral, intakeFlexorRetracted, intakeElbowRetracted);
        setElevator(elevatorLowChamber);
        setOuttakeArm(outtakeElbowBelowChamber, outtakeFlexorBelowChamber, outtakePronatorChamber);
    }

    @Override
    public void loop() {
        drive();
        controlExtendo(gamepad1.y, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.a);
        controlIntake(gamepad1.right_bumper, gamepad1.left_bumper, gamepad1.left_trigger > 0.1, gamepad1.right_trigger > 0.1, gamepad1.x, gamepad1.b, gamepad1.left_stick_button);
        controlOuttake(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.touchpad, gamepad1.touchpad, gamepad1.left_trigger > 0.1, gamepad1.right_trigger > 0.1, gamepad1.a);
        telemetry.addLine("outtake state: " + elevatorState);
        telemetry.update();
    }

    private void drive() {
        // Get joystick values
        float x1 = gamepad1.left_stick_x;   // Strafe (left/right)
        float y1 = -gamepad1.left_stick_y;  // Forward/backward (inverted Y-axis)
        float x2 = (float) (gamepad1.right_stick_x * 0.6);  // Rotation (clockwise/counterclockwise)

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

    private void controlExtendo(boolean activate, boolean forward, boolean back, boolean retract){
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
                    setElevator(elevatorLowChamber);
                    setOuttakeArm(outtakeElbowTransfer, outtakeFlexorTransfer, outtakePronatorTransfer);
                }
                if (activate) {
                    extendoState = ExtendoState.ACTIVE;
                    setIntake(intakePronatorNeutral, intakeFlexorHover, intakeElbowHover);
                    extendoPosition = extendoActive;
                    intakeGrabber.setPosition(intakeGrabberClosed);
                }
            case RETRACTED:
                if (activate) {
                    extendoState = ExtendoState.ACTIVE;
                    setIntake(intakePronatorNeutral, intakeFlexorHover, intakeElbowHover);
                    extendoPosition = extendoActive;
                    intakeGrabber.setPosition(intakeGrabberClosed);
                }
                break;
        }
        extendoRetractPrev = retract;
        extendoBackPrev = back;
        extendoForwardPrev = forward;

        setExtendo(extendoPosition);
    }

    private void controlIntake(boolean forward, boolean reverse, boolean open, boolean close, boolean left, boolean right, boolean passthrough){
        if (forward) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else if (reverse) {
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        } else  {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }

        if (open) {
            intakeGrabber.setPosition(intakeGrabberNeutral);
        } else if (close) {
            intakeGrabber.setPosition(intakeGrabberClosed);
        }

        if (left && right){
            intakePronator.setPosition(intakePronatorNeutral);
        } else if (left && !leftAndRightPrev){
            intakePronator.setPosition(intakePronatorLeft);
        } else if (right && !leftAndRightPrev) {
            intakePronator.setPosition(intakePronatorRight);
        }
        leftAndRightPrev = left || right;

        if (passthrough && extendoState == ExtendoState.RETRACTED) {
            intakeElbow.setPosition(intakeElbowRetracted);
            setElevator(elevatorHighChamber);
        }
    }

    // TODO: outtake pronator
    private void controlOuttake(boolean up, boolean down, boolean transfer, boolean score, boolean open, boolean close, boolean goToTransfer){
        if (goToTransfer && extendoState != ExtendoState.RETRACTED) {
            elevatorState = ElevatorState.RETRACTED;
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
                    swingOutAvailable = false;
                    swingOutTimeStamp = getRuntime();
                    elevatorState = ElevatorState.LOWCHAMBER;
                    setElevator(elevatorLowChamber);
                }
                if (transfer && transferAvailable) {
                    transferAvailable = false;
                    transferTimeStamp = getRuntime();
                    extendoPosition = extendoTransfer - extendoTransferAmount;
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
                    swingOutAvailable = true;
                }
                if (up && !upPrev) {
                    elevatorState = ElevatorState.HIGHCHAMBER;
                    setElevator(elevatorHighChamber);
                    setOuttakeArm(outtakeElbowAboveChamber, outtakeFlexorAboveChamber, outtakePronatorChamber);
                }
                if (down && !downPrev) {
                    elevatorState = ElevatorState.RETRACTED;
                }
                if (score) {
                    setElevator(elevatorLowChamber - elevatorScoring);
                    setOuttakeArm(outtakeElbowBelowChamber, outtakeFlexorBelowChamber, outtakePronatorChamber);
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
                if (score) {
                    setElevator(elevatorHighChamber - elevatorScoring);
                    setOuttakeArm(outtakeElbowBelowChamber, outtakeFlexorBelowChamber, outtakePronatorChamber);
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

    private void openOuttakeGrabber(){
        outtakeGrabberLeft.setPosition(outtakeGrabberLeftOpen);
        outtakeGrabberRight.setPosition(outtakeGrabberRightOpen);
    }

    private void closeOuttakeGrabber(){
        outtakeGrabberLeft.setPosition(outtakeGrabberLeftClose);
        outtakeGrabberRight.setPosition(outtakeGrabberRightClose);
    }
}
