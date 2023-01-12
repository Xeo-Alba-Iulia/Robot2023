package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotHardware {

    private final OpMode myOpMode;   // gain access to methods in the calling OpMode.
    //magnum pipirig ringabel

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally

    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    // Sisteme
    public DcMotor ridicare = null;
    public CRServo vfb1;
    public CRServo vfb2;
    public CRServo claw = null;
    public ElapsedTime timer = new ElapsedTime();

    // Sensors
    public DigitalChannel touch;

    // Constants & Variables
    public double targetPos = 0;
    public final double ridicarePos1 = 300;
    public final double ridicarePos2 = 600;
    public final double ridicarePos3 = 710;
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

//      Sasiu
        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, "MotorFrontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "MotorFrontRight");
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, "MotorBackLeft");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "MotorBackRight");
        claw=myOpMode.hardwareMap.get(CRServo.class, "ServoGheara");

//      Sisteme
        ridicare = myOpMode.hardwareMap.get(DcMotor.class, "MotorRidicare");
        vfb1 = myOpMode.hardwareMap.get(CRServo.class, "ServoVFB1");
        vfb2 = myOpMode.hardwareMap.get(CRServo.class, "ServoVFB2");

//      Sensors
        touch = myOpMode.hardwareMap.get(DigitalChannel.class, "TaciDinGura");
        touch.setMode(DigitalChannel.Mode.INPUT);

        ridicare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicare.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ridicare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        vfb1.setDirection(DcMotorSimple.Direction.REVERSE);

        timer.reset();
    }


    public void setRlidicarePos(Gamepad gamepad1) {
        if(gamepad1.dpad_down)
            targetPos = 0;
        else if(gamepad1.dpad_left)
            targetPos = ridicarePos1;
        else if(gamepad1.dpad_up)
            targetPos = ridicarePos2;
        else if(gamepad1.dpad_right)
            targetPos = ridicarePos3;
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
        } else if (!gamepad1.right_bumper) {
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


}
