package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this OpMode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class CenterstageTeleOp extends LinearOpMode {
    private Blinker control_Hub;
    private Gyroscope imu;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor armMotor;
    private DcMotor elevatorMotor;
    private DcMotor intakeMotor;

    TouchSensor downStopButton;
    TouchSensor upStopButton;

    DistanceSensor rightDistance;
    DistanceSensor leftDistance;

    Servo pixelClampServo;
    Servo droneAngleServo;
    Servo droneLaunchServo;
    Servo boxAngleServo;
    Servo launchServo2;

    //private CRServo elevatorMotor;
    @Override
    public void runOpMode() {

        int armPosition = 0;
        int elevatorPosition = 0;
        double magicBoxPosition = .402;
        double nosePosition = .150;

        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        pixelClampServo = hardwareMap.get(Servo.class, "pixelClampServo");
        droneAngleServo = hardwareMap.get(Servo.class, "droneAngleServo");
        droneLaunchServo = hardwareMap.get(Servo.class, "droneLaunchServo");
        boxAngleServo = hardwareMap.get(Servo.class, "boxAngleServo");
        launchServo2 = hardwareMap.get(Servo.class, "launchServo2");

        //elevatorMotor = hardwareMap.get(CRServo.class, "elevatorMotor");

        downStopButton = hardwareMap.get(TouchSensor.class, "downStopButton");
        upStopButton = hardwareMap.get(TouchSensor.class, "upStopButton");

        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boxAngleServo.setPosition(magicBoxPosition);
        pixelClampServo.setPosition(1);
        droneLaunchServo.setPosition(0);
        launchServo2.setPosition(.323);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Box Position", boxAngleServo.getPosition());
        telemetry.addData("Clamp Position", pixelClampServo.getPosition());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y * 0.8;
            double x = -gamepad1.left_stick_x * 1.1 * 0.8;
            double rx = gamepad1.right_stick_x * 1;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            if (gamepad1.dpad_left) {
                motorFrontLeft.setPower(.8);
                motorBackLeft.setPower(-.8);
                motorFrontRight.setPower(-.8);
                motorBackRight.setPower(.8);
            }
            else if (gamepad1.dpad_right) {
                motorFrontLeft.setPower(-.8);
                motorBackLeft.setPower(.8);
                motorFrontRight.setPower(.8);
                motorBackRight.setPower(-.8);
            }
            else if (gamepad1.dpad_up) {
                motorFrontLeft.setPower(.8);
                motorBackLeft.setPower(.8);
                motorFrontRight.setPower(.8);
                motorBackRight.setPower(.8);
            }
            else if (gamepad1.dpad_down) {
                motorFrontLeft.setPower(-.8);
                motorBackLeft.setPower(-.8);
                motorFrontRight.setPower(-.8);
                motorBackRight.setPower(-.8);
            }
            else {
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);
            }
            if (gamepad1.left_bumper) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad1.right_bumper) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad2.b && armPosition <= 650) {
                armPosition = armPosition + 2;
                magicBoxPosition = magicBoxPosition + .002;
                armMotor.setTargetPosition(armPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.7);
                boxAngleServo.setPosition(magicBoxPosition);
                pixelClampServo.setPosition(.7);

            }

            if (gamepad2.x && armPosition >0) {
                armPosition = armPosition - 2;
                magicBoxPosition = magicBoxPosition - .002;
                armMotor.setTargetPosition(armPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.7);
                boxAngleServo.setPosition(magicBoxPosition);
                if (armPosition < 90){
                    magicBoxPosition = .402;
                    pixelClampServo.setPosition(1);
                    boxAngleServo.setPosition(magicBoxPosition);
                }
            }

            if (gamepad2.y) {
                magicBoxPosition = .700;
                armPosition = 540;
                pixelClampServo.setPosition(.7);
                armMotor.setTargetPosition(armPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.7);
                boxAngleServo.setPosition(magicBoxPosition);
                    /*sleep(2000);
                    armMotor.setTargetPosition(537);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(.7);*/
            }

            if (gamepad2.a && downStopButton.isPressed()) {
                magicBoxPosition = .402;
                armPosition = 0;
                pixelClampServo.setPosition(1);
                armMotor.setTargetPosition(armPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(.7);
                boxAngleServo.setPosition(magicBoxPosition);
            }

            if (gamepad2.back) {
                pixelClampServo.setPosition(1);
            }

            if (gamepad2.dpad_up && !upStopButton.isPressed()) {
                if(armPosition <50){
                    armPosition = 50;
                    armMotor.setTargetPosition(armPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(.7);
                }
                elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                //launchServo2.setPosition(.125);
                elevatorMotor.setPower(1);
            } else {
                //launchServo2.setPosition(.323);
                elevatorMotor.setPower(0);
            }

            if (gamepad2.dpad_down && !downStopButton.isPressed()) {
                elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //launchServo2.setPosition(.125);
                elevatorMotor.setPower(-.8);
            } else {
                //launchServo2.setPosition(.323);
                elevatorMotor.setPower(0);
            }

            if (downStopButton.isPressed()){
                launchServo2.setPosition(.323);
            } else {
                launchServo2.setPosition(.125);
            }

            if (gamepad2.right_bumper) {
                magicBoxPosition = magicBoxPosition + .001;
                boxAngleServo.setPosition(magicBoxPosition);
            }

            if (gamepad2.left_bumper) {
                magicBoxPosition = magicBoxPosition - .001;
                boxAngleServo.setPosition(magicBoxPosition);
            } if (magicBoxPosition <= .402) {
                magicBoxPosition = .402;
            }

            if (gamepad1.guide) {
                droneLaunchServo.setPosition(.5);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Box Position", boxAngleServo.getPosition());
            telemetry.addData("Clamp Position", pixelClampServo.getPosition());
            telemetry.addData("Launch Position", droneLaunchServo.getPosition());
            telemetry.addData("Snot Rocket Position", launchServo2.getPosition());
            telemetry.addData("Elevator Behavior", elevatorMotor.getZeroPowerBehavior());
            telemetry.addData("Right Side Distance", rightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Side Distance", leftDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }
    }
}
