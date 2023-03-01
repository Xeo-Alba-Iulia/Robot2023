package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class PozitiiEncoder extends LinearOpMode {
    private DcMotorEx leftEncoder, rightEncoder, frontEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = hardwareMap.get(DcMotorEx.class, "MotorFrontLeft");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "MotorBackRight");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "MotorBackLeft");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while(!isStopRequested()) {
            telemetry.addData("left encoder", leftEncoder.getCurrentPosition());
            telemetry.addData("right encoder", rightEncoder.getCurrentPosition());
            telemetry.addData("front encoder", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
