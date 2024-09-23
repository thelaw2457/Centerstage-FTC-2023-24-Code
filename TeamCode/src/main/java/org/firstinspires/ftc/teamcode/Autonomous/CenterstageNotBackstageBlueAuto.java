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
@Autonomous (name = "_CenterstageNotBackstageBlueAuto")

public class CenterstageNotBackstageBlueAuto extends LinearOpMode {

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

            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //forward
            motorFrontLeft.setTargetPosition(3200);
            motorBackLeft.setTargetPosition(3200);
            motorFrontRight.setTargetPosition(3200);
            motorBackRight.setTargetPosition(3200);
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

            //tur n (right)
            motorFrontLeft.setTargetPosition(-1500);
            motorBackLeft.setTargetPosition(-1500);
            motorFrontRight.setTargetPosition(1500);
            motorBackRight.setTargetPosition(1500);
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

            //anotha forward
            motorFrontLeft.setTargetPosition(4250);
            motorBackLeft.setTargetPosition(4250);
            motorFrontRight.setTargetPosition(4250);
            motorBackRight.setTargetPosition(4250);
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

            //slide to da right
            motorFrontLeft.setTargetPosition(1600);
            motorBackLeft.setTargetPosition(-1600);
            motorFrontRight.setTargetPosition(-1600);
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

            //lift da arm
            motorFrontLeft.setTargetPosition(1600);
            motorBackLeft.setTargetPosition(-1600);
            motorFrontRight.setTargetPosition(-1600);
            motorBackRight.setTargetPosition(1600);
            pixelClampServo.setPosition(.7);
            armMotor.setTargetPosition(500);
            boxAngleServo.setPosition(1);
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

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy() || armMotor.isBusy()) {
                telemetry.addData("status", "waiting for step to complete");
            }

            //score all da pixel
            motorFrontLeft.setTargetPosition(1600);
            motorBackLeft.setTargetPosition(-1600);
            motorFrontRight.setTargetPosition(-1600);
            motorBackRight.setTargetPosition(1600);
            pixelClampServo.setPosition(1);
            armMotor.setTargetPosition(500);
            boxAngleServo.setPosition(1);
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

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy() || armMotor.isBusy()) {
                telemetry.addData("status", "waiting for step to complete");
            }

            sleep(1000);

            //put down da arm
            motorFrontLeft.setTargetPosition(1600);
            motorBackLeft.setTargetPosition(-1600);
            motorFrontRight.setTargetPosition(-1600);
            motorBackRight.setTargetPosition(1600);
            pixelClampServo.setPosition(.7);
            armMotor.setTargetPosition(0);
            boxAngleServo.setPosition(.588);
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

            while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy() || armMotor.isBusy()) {
                telemetry.addData("status", "waiting for step to complete");
            }
        }
    }
}

