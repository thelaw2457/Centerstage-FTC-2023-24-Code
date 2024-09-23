package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "_CenterstageRedSpikeAuto")

public class CenterstageRedSpikeAuto extends LinearOpMode {

    Servo boxAngleServo;
    Servo pixelClampServo;

    DistanceSensor rightDistance;
    DistanceSensor leftDistance;

    private DcMotor intakeMotor;

    public void runOpMode() {
        int Motor_Velocity;
        double left_Distance;
        double right_Distance;
        int spikePos;

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        boxAngleServo = hardwareMap.get(Servo.class, "boxAngleServo");
        pixelClampServo = hardwareMap.get(Servo.class, "pixelClampServo");

        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");


        Motor_Velocity = 500;
        left_Distance = leftDistance.getDistance(DistanceUnit.INCH);
        right_Distance = rightDistance.getDistance(DistanceUnit.INCH);

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

            telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();

            //init forward
            motorFrontLeft.setTargetPosition(1480);
            motorBackLeft.setTargetPosition(1480);
            motorFrontRight.setTargetPosition(1480);
            motorBackRight.setTargetPosition(1480);
            intakeMotor.setPower(0);
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
                telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }

            //sleep
            sleep(1000);

            if (leftDistance.getDistance(DistanceUnit.INCH) < 4) {
                spikePos = 1;
            } else if (rightDistance.getDistance(DistanceUnit.INCH) < 4) {
                spikePos = 3;
            } else {
                spikePos = 2;
            }

            //read distance
            if (spikePos == 1) {

                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //reverse
                motorFrontLeft.setTargetPosition(-300);
                motorBackLeft.setTargetPosition(-300);
                motorFrontRight.setTargetPosition(-300);
                motorBackRight.setTargetPosition(-300);
                intakeMotor.setPower(0);
                pixelClampServo.setPosition(1);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

                //turn left
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(-1000);
                motorBackLeft.setTargetPosition(-1000);
                motorFrontRight.setTargetPosition(1000);
                motorBackRight.setTargetPosition(1000);
                intakeMotor.setPower(0);
                pixelClampServo.setPosition(1);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

                //forward
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(200);
                motorBackLeft.setTargetPosition(200);
                motorFrontRight.setTargetPosition(200);
                motorBackRight.setTargetPosition(200);
                intakeMotor.setPower(0);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //deposit pixel
                motorFrontLeft.setTargetPosition(0);
                motorBackLeft.setTargetPosition(0);
                motorFrontRight.setTargetPosition(0);
                motorBackRight.setTargetPosition(0);
                intakeMotor.setPower(.7);
                pixelClampServo.setPosition(1);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) motorFrontLeft).setPower(.7);
                ((DcMotorEx) motorBackLeft).setPower(.7);
                ((DcMotorEx) motorFrontRight).setPower(.7);
                ((DcMotorEx) motorBackRight).setPower(.7);

                sleep(500);

                while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                    telemetry.addData("status", "waiting for step to complete");
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

                //reverse
                motorFrontLeft.setTargetPosition(-300);
                motorBackLeft.setTargetPosition(-300);
                motorFrontRight.setTargetPosition(-300);
                motorBackRight.setTargetPosition(-300);
                intakeMotor.setPower(0);
                pixelClampServo.setPosition(1);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

                //read other distance
            } else if (spikePos == 3) {

                //back up

                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(-200);
                motorBackLeft.setTargetPosition(-200);
                motorFrontRight.setTargetPosition(-200);
                motorBackRight.setTargetPosition(-200);
                intakeMotor.setPower(0);
                pixelClampServo.setPosition(1);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

                //slightly right

                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(1000);
                motorBackLeft.setTargetPosition(1000);
                motorFrontRight.setTargetPosition(-1000);
                motorBackRight.setTargetPosition(-1000);
                intakeMotor.setPower(0);
                pixelClampServo.setPosition(1);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

                //forward
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(50);
                motorBackLeft.setTargetPosition(50);
                motorFrontRight.setTargetPosition(50);
                motorBackRight.setTargetPosition(50);
                intakeMotor.setPower(0);
                pixelClampServo.setPosition(1);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }

                //deposit pixel
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(0);
                motorBackLeft.setTargetPosition(0);
                motorFrontRight.setTargetPosition(0);
                motorBackRight.setTargetPosition(0);
                intakeMotor.setPower(1);
                pixelClampServo.setPosition(1);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) motorFrontLeft).setPower(.7);
                ((DcMotorEx) motorBackLeft).setPower(.7);
                ((DcMotorEx) motorFrontRight).setPower(.7);
                ((DcMotorEx) motorBackRight).setPower(.7);

                sleep(350);

                while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                    telemetry.addData("status", "waiting for step to complete");
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position", spikePos);
                    telemetry.update();
                }

                //reverse
                motorFrontLeft.setTargetPosition(-300);
                motorBackLeft.setTargetPosition(-300);
                motorFrontRight.setTargetPosition(-300);
                motorBackRight.setTargetPosition(-300);
                intakeMotor.setPower(0);
                pixelClampServo.setPosition(1);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

            } else {

                //strafe left
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(-300);
                motorBackLeft.setTargetPosition(300);
                motorFrontRight.setTargetPosition(300);
                motorBackRight.setTargetPosition(-300);
                intakeMotor.setPower(0);
                pixelClampServo.setPosition(1);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

                //deposit pixel
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(0);
                motorBackLeft.setTargetPosition(0);
                motorFrontRight.setTargetPosition(0);
                motorBackRight.setTargetPosition(0);
                intakeMotor.setPower(1);
                pixelClampServo.setPosition(1);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) motorFrontLeft).setPower(.7);
                ((DcMotorEx) motorBackLeft).setPower(.7);
                ((DcMotorEx) motorFrontRight).setPower(.7);
                ((DcMotorEx) motorBackRight).setPower(.7);

                sleep(250);
                while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() ||motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                    telemetry.addData("status", "waiting for step to complete");
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position", spikePos);
                    telemetry.update();
                }

                //reverse
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                motorFrontLeft.setTargetPosition(-300);
                motorBackLeft.setTargetPosition(-300);
                motorFrontRight.setTargetPosition(-300);
                motorBackRight.setTargetPosition(-300);
                intakeMotor.setPower(0);
                pixelClampServo.setPosition(1);
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
                    telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("Spike Position" , spikePos);
                    telemetry.update();
                }

            }




            telemetry.addData("FL Position", motorFrontLeft.getCurrentPosition());
            telemetry.addData("FR Position", motorFrontRight.getCurrentPosition());
            telemetry.addData("BL Position", motorBackLeft.getCurrentPosition());
            telemetry.addData("BR Position", motorBackRight.getCurrentPosition());
            telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Spike Position" , spikePos);
            telemetry.update();




        }


    }

}

