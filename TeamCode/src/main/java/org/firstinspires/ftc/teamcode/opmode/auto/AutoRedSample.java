package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutoConstants.positionThreshold;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Disabled
@Autonomous
public class AutoRedSample extends OpMode {
    private DcMotorEx extendo;
    private DcMotorEx elevatorLeft, elevatorRight;

    // Declare servos
    private CRServo intakeLeft, intakeRight;
    private Servo intakeGrabber, intakePronator, intakeFlexor, intakeElbow;
    private Servo outtakePronator, outtakeFlexor, outtakeElbowLeft, outtakeElbowRight, outtakeGrabberLeft, outtakeGrabberRight;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /**
     * stores state of auto
     * used by pathUpdate method
     */
    private int pathState;

    // state machines

    /**
     * create/define poses & paths
     * poses have x, y, heading
     * 0-144 for (x, y); (0, 0) is the bottom left
     * blue observation zone (0, 0) and red observation zone (144, 144)
     * convert RR to pedro by adding 72 to x and y
     * <<a href="https://pedro-path-generator.vercel.app/">...</a>>
     * FYI outtake is the front of the bot
     */

    // robot starts facing the basket
    private final Pose startPose = new Pose(137.5, 30.5, Math.toRadians(225));

    // scoring on basket - intake faces the spike marks
    private final Pose basketScorePose = new Pose(120, 12, Math.toRadians(345));

    // first (wall) sample from spike mark
    private final Pose intakeSample1Pose = new Pose(96, 12, Math.toRadians(15));

    // second sample from spike mark
    private final Pose intakeSample2Pose = new Pose(120, 12, Math.toRadians(0));

    // third sample from spike amrk
    private final Pose intakeSample3Pose = new Pose(120, 12, Math.toRadians(345));

    // parking
    private final Pose parkPose = new Pose(84, 40, Math.toRadians(90));

    /**
     * parking control pose to manipulate curve
     * robot will not go to this pose, it is a control point for the curve
     */
    private final Pose parkControlPose = new Pose(84, 12, Math.toRadians(90));

    // paths and pathchains that will be defined in buildPaths()
//    private Path scorePreload, park;
    private PathChain scorePreload, intakeSample1, intakeSample2, intakeSample3, scoreSample1, scoreSample2, scoreSample3, park;

    // timer
    private double timestamp = 0;
    private boolean timerActive = false;

    // build paths for auto
    public void buildPaths(){
        // straight line to basket to score preload
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(basketScorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), basketScorePose.getHeading())
                .build();
//        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(basketScorePose)));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), basketScorePose.getHeading());

        // intakeSample1 PathChain with a single path, straight line
        intakeSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketScorePose), new Point(intakeSample1Pose)))
                .setLinearHeadingInterpolation(basketScorePose.getHeading(), intakeSample1Pose.getHeading())
                .build();

        // scoreSample1 PathChain with a single path, straight line
        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeSample1Pose), new Point(basketScorePose)))
                .setLinearHeadingInterpolation(intakeSample1Pose.getHeading(), basketScorePose.getHeading())
                .build();

        // intakeSample2 PathChain with a single path, straight line
        intakeSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketScorePose), new Point(intakeSample2Pose)))
                .setLinearHeadingInterpolation(basketScorePose.getHeading(), intakeSample2Pose.getHeading())
                .build();

        // scoreSample2 PathChain with a single path, straight line
        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeSample2Pose), new Point(basketScorePose)))
                .setLinearHeadingInterpolation(intakeSample2Pose.getHeading(), basketScorePose.getHeading())
                .build();

        // intakeSample3 PathChain with a single path, straight line
        intakeSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketScorePose), new Point(intakeSample3Pose)))
                .setLinearHeadingInterpolation(basketScorePose.getHeading(), intakeSample3Pose.getHeading())
                .build();

        // scoreSample3 PathChain with a single path, straight line
        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeSample3Pose), new Point(basketScorePose)))
                .setLinearHeadingInterpolation(intakeSample3Pose.getHeading(), basketScorePose.getHeading())
                .build();

        // parking path, BezierCurve with 3 points, curve is based on the control point
        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(basketScorePose), new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(basketScorePose.getHeading(), parkPose.getHeading())
                .build();
//        park = new Path(new BezierCurve(new Point(basketScorePose), new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(basketScorePose.getHeading(), parkPose.getHeading());
    }

    /**
     * this switch is called continuously and runs the pathing
     * at certain points it triggers the action state
     * timer is reset every time the switch changes case (due to setPathState() method)
     * followPath() function sets follower to run the specific path, but does NOT wait for it to finish before moving on
     */
    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                /**
                 * You could check for:
                 * Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                 * Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                 * Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                // this case checks robot's position and waits to get close enuogh
                if ((Math.abs(follower.getPose().getX() - basketScorePose.getX()) < positionThreshold) && (Math.abs(follower.getPose().getY() - basketScorePose.getY()) < positionThreshold)){
                    // score preload

                    // wait
                    if (!timerActive){
                        timestamp = pathTimer.getElapsedTime();
                        timerActive = true;
                    }
                    if (pathTimer.getElapsedTime() - timestamp > 2000){
                        // as this is PathChain, endpoint is held while grabbing sample
                        follower.followPath(intakeSample1, true);
                        setPathState(2);
                        timestamp = pathTimer.getElapsedTime();
                        timerActive = false;
                    }
                }
                break;
            case 2:
                // checks robot's position and waits to get close enough
                if (Math.abs(follower.getPose().getX() - intakeSample1Pose.getX()) < 1 && Math.abs(follower.getPose().getY() - intakeSample1Pose.getY()) < 1){
                    // grab sample

                    // wait
                    if (!timerActive){
                        timestamp = pathTimer.getElapsedTime();
                        timerActive = true;
                    }
                    if (pathTimer.getElapsedTime() - timestamp > 2000){
                        // hold endpoint while scoring sample
                        follower.followPath(scoreSample1, true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                // checks robot's position and waits to get close enough
                if (Math.abs(follower.getPose().getX() - basketScorePose.getX()) < 1 && Math.abs(follower.getPose().getY() - basketScorePose.getY()) < 1) {
                    // score sample

                    // wait
                    if (!timerActive){
                        timestamp = pathTimer.getElapsedTime();
                        timerActive = true;
                    }
                    if (pathTimer.getElapsedTime() - timestamp > 2000){
                        // hold endpoint while parked
                        follower.followPath(park, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                // checks robot's position and waits to get close enough
                if (Math.abs(follower.getPose().getX() - parkPose.getX()) < 1 && Math.abs(follower.getPose().getY() - parkPose.getY()) < 1) {
                    // move outtake to get level 1 ascent

                    // set path state to undefined case to stop running
                    setPathState(-1);
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
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

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
