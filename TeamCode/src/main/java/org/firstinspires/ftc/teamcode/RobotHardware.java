package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class RobotHardware {

    private final OpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define a constructor that allows the OpMode to pass a reference to itself.


    // Define Motor and Servo objects  (Make them private so they can't be accessed externally
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    // Sisteme
    public DcMotor fourbar1 = null;
    public DcMotor fourbar2 = null;
    public CRServo claw = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, "MotorFrontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "MotorFrontRight");
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, "MotorBackLeft");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "MotorBackRight");
        fourbar1 = myOpMode.hardwareMap.get(DcMotor.class, "MotorFourbar");
        claw = myOpMode.hardwareMap.get(CRServo.class, "ServoGheara");

        fourbar1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourbar2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fourbar1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void movement(Gamepad gamepad1) {
        double frontLeftPower = gamepad1.left_stick_y -
                gamepad1.left_stick_x -
                gamepad1.right_stick_x;
        double backLeftPower = gamepad1.left_stick_y +
                gamepad1.left_stick_x -
                gamepad1.right_stick_x;
        double frontRightPower = -gamepad1.left_stick_y -
                gamepad1.left_stick_x -
                gamepad1.right_stick_x;
        double backRightPower = -gamepad1.left_stick_y +
                gamepad1.left_stick_x -
                gamepad1.right_stick_x;
        if (gamepad1.left_bumper) {
            frontRightPower *= 0.3;
            frontLeftPower *= 0.3;
            backRightPower *= 0.3;
            backLeftPower *= 0.3;
        } else if (!gamepad1.right_bumper && gamepad1.left_trigger == 0) {
            frontRightPower *= 0.6;
            frontLeftPower *= 0.6;
            backRightPower *= 0.6;
            backLeftPower *= 0.6;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void setFourbarPower(double power) {
        fourbar1.setPower(power);
        fourbar2.setPower(power);
    }

}
