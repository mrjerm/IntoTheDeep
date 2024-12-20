package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.opmode.Constants_ITD.elevatorBasket;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class MotorEncoderDead extends OpMode {
    private DcMotorEx motor1;
    private DcMotorEx motor2;
//    private DcMotorEx motor3;
    private DcMotorEx motor4;

    boolean upPrev = false;
    boolean downPrev = false;
    boolean leftPrev = false;
    boolean rightPrev = false;
    int position = 0;
    int position2 =0;
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "Motor Elevator L");
        motor2 = hardwareMap.get(DcMotorEx.class, "Motor Elevator R");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);


        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        motor3 = hardwareMap.get(DcMotorEx.class, "Motor Stick");
//        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
        motor4 = hardwareMap.get(DcMotorEx.class, "Motor Extendo");
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setPower(0);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !upPrev){
            position += 100;
        }
        upPrev = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !downPrev) {
            position -= 100;
        }
        downPrev = gamepad1.dpad_down;
        position = Math.max(0, position);
        motor1.setTargetPosition(position);
        motor2.setTargetPosition(position);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(1);
        motor2.setPower(1);
//
//
//
//        if (gamepad1.dpad_right && !rightPrev) {
//            position2+=10;
//        }
//        if (gamepad1.dpad_left && !leftPrev) {
//            position2-=10;
//        }
//        rightPrev = gamepad1.dpad_right;
//        leftPrev = gamepad1.dpad_left;
//        motor3.setTargetPosition(position2);
//        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor3.setPower(1);
//
//
//        motor4.setTargetPosition(0);
//        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor4.setPower(1);
//
//        telemetry.addLine("stick: " + motor3.getCurrentPosition());
        telemetry.addLine("left: " + motor1.getCurrentPosition());
        telemetry.addLine("right: " + motor2.getCurrentPosition());
        telemetry.addLine("extendo: " + motor4.getCurrentPosition());
        telemetry.update();
    }
}
