package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.sisteme.VirtualFourBar;


public class RobotHardware {
    public final double VFB_OUTTAKE_POSE = 0.85;

    public double VFB_STACK_POSE = 0.69;
    //magnum pipirig ringabel

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public final double VFB_ALIGN_POSE = 0.4;
    public final double VFB_INTAKE_POSE = 0.87;

  //  public final double VFB_LOW_FRONT= 0.50;

    public final double VFB_FALLEN= 0.83;

    public final double VFB_LOW= 0.09;

    public final double VFB_MEDIUM= 0.13;

    public final double VFB_HIGH= 0.27;
    public final double VFB_INTER= 0.35;

    public final double CLAW_ALLIGN_POS_UP = 0.15;

    public final double CLAW_ALLIGN_POS_LOW= 0.6;

    public final double CLAW_ALLIGN_POS_HIGH = 0.49;
    public final double CLAW_ALLIGN_POS_INTAKE = 0.47;
    public final double CLAW_ALLIGN_POS_FALLEN = 0.85;

    public final double CLAW_ALLIGN_POS_INTER = 0.15;
    public final double GHEARA_DESCHISA = 0.72;
    public final double GHEARA_INCHISA = 0.6;

    public final double GHEARA_INIT = 0.65;
    private final OpMode myOpMode;   // gain access to methods in the calling OpMode.
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;
    // Sisteme
    public DcMotorEx ridicare1 = null;
    public DcMotorEx ridicare2 = null;
    public Servo claw = null;
    public Servo claw_alligner = null;
    public ServoImplEx vFB1 = null;
    public ServoImplEx vFB2 = null;
    public VirtualFourBar virtualFourBar;
    public double vfb_stack_pose ;
    public Ridicare lift;

    // Constants & Variables


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
        lift = new Ridicare(ridicare1, ridicare2);
        claw = myOpMode.hardwareMap.get(Servo.class, "Gheara");
        claw_alligner = myOpMode.hardwareMap.get(Servo.class, "AliniereGheara");
        vFB1 = myOpMode.hardwareMap.get(ServoImplEx.class, "vfb1");
        vFB1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        vFB2 = myOpMode.hardwareMap.get(ServoImplEx.class, "vfb2");
        vFB2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        virtualFourBar = new VirtualFourBar(vFB1, vFB2);



        ridicare1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ridicare1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void movement(@NonNull Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x;
        if (gamepad1.left_stick_x < 0.25 && gamepad1.left_stick_x > -0.25) {
            x = 0;
        } else {
            x = gamepad1.left_stick_x;
        }
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        if (gamepad1.left_trigger!=0) {
            frontRightPower *= 0.4;
            frontLeftPower *= 0.4;
            backRightPower *= 0.4;
            backLeftPower *= 0.4;
        } else if (gamepad1.right_trigger!=0) {
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
