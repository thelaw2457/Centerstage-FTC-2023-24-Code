package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous (name = "_CenterstageRedAuto")

public class CenterstageRedAuto extends LinearOpMode {

    Servo boxAngleServo;
    Servo pixelClampServo;

    public void runOpMode() {
        int Motor_Velocity;

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

        boxAngleServo = hardwareMap.get(Servo.class, "boxAngleServo");
        pixelClampServo = hardwareMap.get(Servo.class, "pixelClampServo");


        Motor_Velocity = 500;

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()) {

            motorFrontLeft.setTargetPosition(-2050);
            motorBackLeft.setTargetPosition(2050);
            motorFrontRight.setTargetPosition(2050);
            motorBackRight.setTargetPosition(-2050);
            pixelClampServo.setPosition(.7);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) motorFrontLeft).setPower(.7);
            ((DcMotorEx) motorBackLeft).setPower(.7);
            ((DcMotorEx) motorFrontRight).setPower(.7);
            ((DcMotorEx) motorBackRight).setPower(.7);

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                telemetry.addData("status", "waiting for step to complete");
            }

            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFrontLeft.setTargetPosition(1600);
            motorBackLeft.setTargetPosition(1600);
            motorFrontRight.setTargetPosition(1600);
            motorBackRight.setTargetPosition(1600);
            pixelClampServo.setPosition(.7);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) motorFrontLeft).setPower(.7);
            ((DcMotorEx) motorBackLeft).setPower(.7);
            ((DcMotorEx) motorFrontRight).setPower(.7);
            ((DcMotorEx) motorBackRight).setPower(.7);

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                telemetry.addData("status", "waiting for step to complete");
            }

            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFrontLeft.setTargetPosition(1550);
            motorBackLeft.setTargetPosition(1550);
            motorFrontRight.setTargetPosition(-1550);
            motorBackRight.setTargetPosition(-1550);
            pixelClampServo.setPosition(.7);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) motorFrontLeft).setPower(.7);
            ((DcMotorEx) motorBackLeft).setPower(.7);
            ((DcMotorEx) motorFrontRight).setPower(.7);
            ((DcMotorEx) motorBackRight).setPower(.7);

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                telemetry.addData("status", "waiting for step to complete");
            }

                /*motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(50);
                motorBackLeft.setTargetPosition(50);
                motorFrontRight.setTargetPosition(50);
                motorBackRight.setTargetPosition(50);
                pixelClampServo.setPosition(.7);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) motorFrontLeft).setPower(.7);
                ((DcMotorEx) motorBackLeft).setPower(.7);
                ((DcMotorEx) motorFrontRight).setPower(.7);
                ((DcMotorEx) motorBackRight).setPower(.7);

                while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                    telemetry.addData("status", "waiting for step to complete");
                }*/

            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFrontLeft.setTargetPosition(0);
            motorBackLeft.setTargetPosition(0);
            motorFrontRight.setTargetPosition(0);
            motorBackRight.setTargetPosition(0);
            pixelClampServo.setPosition(.7);
            armMotor.setTargetPosition(575);
            boxAngleServo.setPosition(.88);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) motorFrontLeft).setPower(.7);
            ((DcMotorEx) motorBackLeft).setPower(.7);
            ((DcMotorEx) motorFrontRight).setPower(.7);
            ((DcMotorEx) motorBackRight).setPower(.7);
            armMotor.setPower(.7);

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy() || armMotor.isBusy()){
                telemetry.addData("status", "waiting for step to complete");
            }

            sleep(2000);

            motorFrontLeft.setTargetPosition(0);
            motorBackLeft.setTargetPosition(0);
            motorFrontRight.setTargetPosition(0);
            motorBackRight.setTargetPosition(0);
            pixelClampServo.setPosition(1);
            armMotor.setTargetPosition(575);
            boxAngleServo.setPosition(.88);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) motorFrontLeft).setPower(.7);
            ((DcMotorEx) motorBackLeft).setPower(.7);
            ((DcMotorEx) motorFrontRight).setPower(.7);
            ((DcMotorEx) motorBackRight).setPower(.7);
            armMotor.setPower(.7);

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy() || armMotor.isBusy()){
                telemetry.addData("status", "waiting for step to complete");
            }

            sleep(2000);

            motorFrontLeft.setTargetPosition(0);
            motorBackLeft.setTargetPosition(0);
            motorFrontRight.setTargetPosition(0);
            motorBackRight.setTargetPosition(0);
            pixelClampServo.setPosition(1);
            armMotor.setTargetPosition(0);
            boxAngleServo.setPosition(.402);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) motorFrontLeft).setPower(.7);
            ((DcMotorEx) motorBackLeft).setPower(.7);
            ((DcMotorEx) motorFrontRight).setPower(.7);
            ((DcMotorEx) motorBackRight).setPower(.7);
            armMotor.setPower(.7);

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy() || armMotor.isBusy()){
                telemetry.addData("status", "waiting for step to complete");
            }

            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFrontLeft.setTargetPosition(-1650);
            motorBackLeft.setTargetPosition(1650);
            motorFrontRight.setTargetPosition(1650);
            motorBackRight.setTargetPosition(-1650);
            pixelClampServo.setPosition(1);
            armMotor.setTargetPosition(0);
            boxAngleServo.setPosition(.402);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) motorFrontLeft).setPower(.7);
            ((DcMotorEx) motorBackLeft).setPower(.7);
            ((DcMotorEx) motorFrontRight).setPower(.7);
            ((DcMotorEx) motorBackRight).setPower(.7);
            armMotor.setPower(.7);

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy() || armMotor.isBusy()){
                telemetry.addData("status", "waiting for step to complete");
            }

            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFrontLeft.setTargetPosition(300);
            motorBackLeft.setTargetPosition(300);
            motorFrontRight.setTargetPosition(300);
            motorBackRight.setTargetPosition(300);
            pixelClampServo.setPosition(1);
            armMotor.setTargetPosition(0);
            boxAngleServo.setPosition(.402);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) motorFrontLeft).setPower(.7);
            ((DcMotorEx) motorBackLeft).setPower(.7);
            ((DcMotorEx) motorFrontRight).setPower(.7);
            ((DcMotorEx) motorBackRight).setPower(.7);
            armMotor.setPower(.7);

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy() || armMotor.isBusy()){
                telemetry.addData("status", "waiting for step to complete");
            }

            telemetry.addData("FL Position", motorFrontLeft.getCurrentPosition());
            telemetry.addData("FR Position", motorFrontRight.getCurrentPosition());
            telemetry.addData("BL Position", motorBackLeft.getCurrentPosition());
            telemetry.addData("BR Position", motorBackRight.getCurrentPosition());
            telemetry.update();




        }


    }

}

