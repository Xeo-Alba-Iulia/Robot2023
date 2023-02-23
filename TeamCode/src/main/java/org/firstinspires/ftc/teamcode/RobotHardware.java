package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.sisteme.Ridicare;


public class RobotHardware {

    private final OpMode myOpMode;   // gain access to methods in the calling OpMode.
    //magnum pipirig ringabel

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)

    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;

    // Sisteme
    public DcMotorEx ridicare1 = null;
    public DcMotorEx ridicare2 = null;
    public Servo claw = null;
    public ServoImplEx vFB1 = null;
    public ServoImplEx vFB2 = null;
    public ElapsedTime ridicareTimer = new ElapsedTime();
    public ElapsedTime clawTimer = new ElapsedTime();
    public final double VFB_OUTTAKE_POSE = 0.85;
    public final double VFB_ALIGN_POSE = 0.5;
    public double vfb_stack_pose;
    public final double VFB_INTAKE_POSE = 0;
    public final double VFB_ALIGN_HIGH_POSE = 0.70;

    public final double GHEARA_DESCHISA = 0.475;
    public final double GHEARA_INCHISA = 0.410;

    public Ridicare lift;

    // Constants & Variables
    public double target = 0;
    public final int RIDICARE_POS_1 = 8600;
    public final int RIDICARE_POS_2 = 16500;
    public final int RIDICARE_POS_3 = 19500;


    // Define a constructor that allows the OpMode to pass a reference to itself.

    public RobotHardware(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
//      Sasiu
        frontLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "MotorFrontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotorEx.class, "MotorFrontRight");
        backLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "MotorBackLeft");
        backRight = myOpMode.hardwareMap.get(DcMotorEx.class, "MotorBackRight");

//      Sisteme
        ridicare1 = myOpMode.hardwareMap.get(DcMotorEx.class, "RidicareAproape");
        ridicare2 = myOpMode.hardwareMap.get(DcMotorEx.class, "RidicareDeparte");
        lift = new Ridicare (ridicare1, ridicare2, myOpMode);
        claw = myOpMode.hardwareMap.get(Servo.class, "ServoGheara");
        vFB1 = myOpMode.hardwareMap.get(ServoImplEx.class, "vfb1");
        vFB1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        vFB2 = myOpMode.hardwareMap.get(ServoImplEx.class, "vfb2");
        vFB2.setPwmRange(new PwmControl.PwmRange(500,2500));

        ridicare1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicare1.setTargetPosition(0);
        ridicare1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ridicare1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ridicare2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicare2.setTargetPosition(0);
        ridicare2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ridicare2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ridicare1.setDirection(DcMotorSimple.Direction.FORWARD);
        ridicare2.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void start() {
        ridicareTimer.reset();
    }

    public void setVFBPosition(double position) {
        vFB1.setPosition(position);
        vFB2.setPosition(1 - position);
    }

    public void movement(Gamepad gamepad1) {

        double y = -gamepad1.left_stick_y;
        double x;
        if(gamepad1.left_stick_x < 0.25 && gamepad1.left_stick_x > -0.25)
            x = 0;
        else
        {
            x=gamepad1.left_stick_x;
        }
        double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

        if (gamepad1.left_bumper) {
            frontRightPower *= 0.4;
            frontLeftPower *= 0.4;
            backRightPower *= 0.4;
            backLeftPower *= 0.4;
        } else if (!gamepad1.right_bumper) {
            frontRightPower *= 0.6;
            frontLeftPower *= 0.6;
            backRightPower *= 0.6;
            backLeftPower *= 0.6;
        }



        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);}

}



