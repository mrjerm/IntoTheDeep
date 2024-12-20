package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorHighChamber;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorPower;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorRetracted;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoMax;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoPower;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.extendoRetracted;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeElbowGrab;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeElbowHover;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeElbowRetracted;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeFlexorGrab;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeFlexorHover;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeFlexorRetracted;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeFlexorVertical;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeGrabberClosed;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakeGrabberOpen;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.intakePronatorNeutral;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeElbowHorizontal;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeElbowInitialize;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeFlexorInitialize;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeFlexorStraight;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeGrabberLeftClose;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeGrabberLeftOpen;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeGrabberRightClose;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakeGrabberRightOpen;
import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.outtakePronatorChamber;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoConstants.positionThreshold;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoConstants.redSpecimen_dropSample3Pose;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoConstants.redSpecimen_scorePreloadPose;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoConstants.redSpecimen_spikeLeftPose;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoConstants.redSpecimen_spikeMiddlePose;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoConstants.redSpecimen_spikeRightPose;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutoConstants.redSpecimen_startPose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

//TODO: change to preload -> park and make one for bucket side
@Disabled
@Autonomous
public class AutoRedSpecimenPreloadOnly extends OpMode {
    private DcMotorEx extendo;
    private DcMotorEx elevatorLeft, elevatorRight;

    // Declare servos
    private Servo intakeGrabber, intakePronator, intakeFlexor, intakeElbow;
    private Servo outtakePronator, outtakeFlexor, outtakeElbowLeft, outtakeElbowRight, outtakeGrabberLeft, outtakeGrabberRight;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /**
     * stores state of auto
     * used by pathUpdate method
     */
    private int pathState;

    // paths and pathchains that will be defined in buildPaths()
//    private Path scorePreload, park;
    private PathChain scorePreload, intakeSpecimen1, intakeSpecimen2, intakeSpecimen3, park;

    // timer
    private double pathTimeStamp = 0;
    private double actionTimeStamp = 0;
    private boolean timerActive = false;
    private boolean debounce = false;
    private void setElevator(int position){
        elevatorLeft.setTargetPosition(position);
        elevatorRight.setTargetPosition(position);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(elevatorPower);
        elevatorRight.setPower(elevatorPower);
    }

    private void setExtendo(int position){
        extendo.setTargetPosition(position);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(extendoPower);
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

    private void setIntake(double pronation, double flexion, double elbow){
        intakePronator.setPosition(pronation);
        intakeFlexor.setPosition(flexion);
        intakeElbow.setPosition(elbow);
    }

    // build paths for auto
    public void buildPaths(){
        // straight line to chamber to score preload
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(redSpecimen_startPose), new Point(redSpecimen_scorePreloadPose)))
                .setLinearHeadingInterpolation(redSpecimen_startPose.getHeading(), redSpecimen_scorePreloadPose.getHeading())
                .build();

        intakeSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(redSpecimen_scorePreloadPose), new Point(redSpecimen_spikeLeftPose)))
                .setLinearHeadingInterpolation(redSpecimen_scorePreloadPose.getHeading(), redSpecimen_spikeLeftPose.getHeading())
                .build();

        intakeSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(redSpecimen_spikeLeftPose), new Point(redSpecimen_spikeMiddlePose)))
                .setLinearHeadingInterpolation(redSpecimen_spikeLeftPose.getHeading(), redSpecimen_spikeMiddlePose.getHeading())
                .build();

        intakeSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(redSpecimen_spikeMiddlePose), new Point(redSpecimen_spikeRightPose)))
                .setLinearHeadingInterpolation(redSpecimen_spikeMiddlePose.getHeading(), redSpecimen_spikeRightPose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(redSpecimen_spikeMiddlePose), new Point(redSpecimen_dropSample3Pose)))
                .setLinearHeadingInterpolation(redSpecimen_spikeMiddlePose.getHeading(), redSpecimen_dropSample3Pose.getHeading())
                .build();
    }

    /**
     * this switch is called continuously and runs the pathing
     * at certain points it triggers the action state
     * timer is reset every time the switch changes case (due to setPathState() method)
     * followPath() function sets follower to run the specific path, but does NOT wait for it to finish before moving on
     */
    //TODO: THIS PART
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                setElevator(elevatorHighChamber);
                setOuttakeArm(outtakeElbowHorizontal, outtakeFlexorStraight, outtakePronatorChamber);

                follower.followPath(scorePreload);
                setPathState(1);
                debounce = true;
                break;
            case 1:
                /**
                 * You could check for:
                 * Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                 * Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                 * Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                // this case checks robot's position and waits to get close enuogh
                if ((Math.abs(follower.getPose().getX() - redSpecimen_scorePreloadPose.getX()) < positionThreshold) && (Math.abs(follower.getPose().getY() - redSpecimen_scorePreloadPose.getY()) < positionThreshold)){
                    // score preload
                    setOuttakeGrabber(true);

                    // wait
                    if (!timerActive){
                        pathTimeStamp = pathTimer.getElapsedTime();
                        actionTimeStamp = actionTimer.getElapsedTime();
                        timerActive = true;
                    }
                    if (pathTimer.getElapsedTime() - pathTimeStamp > 500){
                        // as this is PathChain, endpoint is held while grabbing sample
                        follower.followPath(intakeSpecimen1, true);
                        setPathState(2);
                        pathTimeStamp = pathTimer.getElapsedTime();
                        timerActive = false;
                    }
                }
                debounce = true;
                break;
            case 2:
                // checks robot's position and waits to get close enough
                if (debounce){
                    setExtendo(extendoMax);
                    setIntake(intakePronatorNeutral, intakeFlexorHover, intakeElbowHover - 0.04);
                    debounce = false;
                }

                if (Math.abs(follower.getPose().getX() - redSpecimen_spikeLeftPose.getX()) < positionThreshold && Math.abs(follower.getPose().getY() - redSpecimen_spikeLeftPose.getY()) < positionThreshold){
                    // start timer
                    if (!timerActive){
                        pathTimeStamp = pathTimer.getElapsedTime();
                        actionTimer.resetTimer();
                        timerActive = true;
                    }
                    if (actionTimer.getElapsedTime() >= 600 && actionTimer.getElapsedTime() < 800){
                        setIntake(intakePronatorNeutral, intakeFlexorGrab, intakeElbowGrab);
                    }
                    if (actionTimer.getElapsedTime() >= 800 && actionTimer.getElapsedTime() < 1100){
                        intakeGrabber.setPosition(intakeGrabberClosed);
                    }
                    if (actionTimer.getElapsedTime() >= 1100 && actionTimer.getElapsedTime() < 1300){
                        setIntake(intakePronatorNeutral, intakeFlexorRetracted, intakeElbowRetracted);
                        setExtendo(extendoRetracted);
                    }
                    if (actionTimer.getElapsedTime() >= 2000 && actionTimer.getElapsedTime() < 2200){
                        intakeGrabber.setPosition(intakeGrabberOpen);
                    }
                    if (pathTimer.getElapsedTime() - pathTimeStamp > 2300){
                        // hold endpoint while scoring sample
                        follower.followPath(intakeSpecimen2);
                        setPathState(3);
                        debounce = true;
                        timerActive = false;
                    }
                }
                break;
            case 3:
                // checks robot's position and waits to get close enough
                if (debounce){
                    setExtendo(extendoMax);
                    setIntake(intakePronatorNeutral, intakeFlexorHover, intakeElbowHover - 0.04);
                    debounce = false;
                }

                if (Math.abs(follower.getPose().getX() - redSpecimen_spikeMiddlePose.getX()) < positionThreshold && Math.abs(follower.getPose().getY() - redSpecimen_spikeMiddlePose.getY()) < positionThreshold){
                    // start timer
                    if (!timerActive){
                        pathTimeStamp = pathTimer.getElapsedTime();
                        actionTimer.resetTimer();
                        timerActive = true;
                    }
                    if (actionTimer.getElapsedTime() >= 600 && actionTimer.getElapsedTime() < 800){
                        setIntake(intakePronatorNeutral, intakeFlexorGrab, intakeElbowGrab);
                    }
                    if (actionTimer.getElapsedTime() >= 800 && actionTimer.getElapsedTime() < 1100){
                        intakeGrabber.setPosition(intakeGrabberClosed);
                    }
                    if (actionTimer.getElapsedTime() >= 1100 && actionTimer.getElapsedTime() < 1300){
                        setIntake(intakePronatorNeutral, intakeFlexorRetracted, intakeElbowRetracted);
                        setExtendo(extendoRetracted);
                    }
                    if (actionTimer.getElapsedTime() >= 2000 && actionTimer.getElapsedTime() < 2200){
                        intakeGrabber.setPosition(intakeGrabberOpen);
                    }
                    if (pathTimer.getElapsedTime() - pathTimeStamp > 2300){
                        setExtendo(extendoRetracted);
                        setIntake(intakePronatorNeutral, intakeFlexorVertical, intakeElbowHover);
                        setElevator(elevatorRetracted);
                        setOuttakeArm(outtakeElbowInitialize, outtakeFlexorInitialize, outtakePronatorChamber);
                        setOuttakeGrabber(true);

                        // hold endpoint while scoring sample
                        follower.followPath(park);
                        setPathState(-1);
                        debounce = true;
                        timerActive = false;
                    }
                }
                break;
        }
    }

    /**
     * changes the state of the paths and actions
     * also resets timers of switches
     */
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        // loop robot movements
        follower.update();
        autonomousPathUpdate();

        // feedback to driver hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Pronator", intakePronator.getPosition());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(redSpecimen_startPose);

        buildPaths();

        // Initialize the motors
        extendo = hardwareMap.get(DcMotorEx.class, "Motor Extendo");
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "Motor Elevator L");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "Motor Elevator R");
        
        // Set motors
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        // Move to initialization configuration
        setElevator(elevatorRetracted);
        setOuttakeArm(outtakeElbowInitialize, outtakeFlexorInitialize, outtakePronatorChamber);
        setOuttakeGrabber(false);
        setExtendo(extendoRetracted);
        setIntake(intakePronatorNeutral, intakeFlexorRetracted, intakeElbowRetracted);
        intakeGrabber.setPosition(intakeGrabberOpen);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }



}
