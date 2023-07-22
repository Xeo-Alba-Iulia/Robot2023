package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.WHEEL_RADIUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "its 1 am i want to sleep")
public class TunerSmecherie extends LinearOpMode {

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * 1 * ticks / TICKS_PER_REV;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        Pose2d startPose = new Pose2d(0, 0 ,Math.toRadians(90));
        int NUM = 10;
        DcMotorEx leftEncoder = hardwareMap.get(DcMotorEx.class, "MotorBackRight");
        DcMotorEx rightEncoder = hardwareMap.get(DcMotorEx.class, "MotorBackLeft");
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MovingStatistics statsRight = new MovingStatistics(NUM);
        MovingStatistics statsLeft = new MovingStatistics(NUM);
        double leftTravel = 0;
        double rightTravel = 0;
        waitForStart();
        drive.setPoseEstimate(startPose);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR)
        ));
        double lastLeft = 0;
        double lastRight = 0;
        for (int i = 0; i < NUM; i++) {

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;
            drive.turnAsync(Math.toRadians(180));


            while (!isStopRequested() && drive.isBusy()) {
                double externalHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                headingAccumulator += Angle.normDelta(externalHeading - lastHeading);
                lastHeading = externalHeading;
                drive.update();

            }
            rightTravel = encoderTicksToInches(rightEncoder.getCurrentPosition()) - lastRight;
            leftTravel = encoderTicksToInches(leftEncoder.getCurrentPosition()) - lastLeft;
            lastRight =encoderTicksToInches(rightEncoder.getCurrentPosition());
            lastLeft = encoderTicksToInches(leftEncoder.getCurrentPosition());
            double resultRight = rightTravel / headingAccumulator;
            double resultLeft = leftTravel / headingAccumulator;
            statsRight.add(resultRight);
            statsLeft.add(resultLeft);
            telemetry.update();
            telemetry.addData("Acumulator rads", headingAccumulator);
            telemetry.addData("Acumulator deg", Math.toDegrees(headingAccumulator));
            telemetry.addData("Distanta Stanga", leftTravel);
            telemetry.addData("Distanta Dreapta", rightTravel);
            telemetry.update();
            sleep(1000);
        }
        telemetry.addData("raza dreapta", statsRight.getMean());
        telemetry.addData("raza stanga", statsLeft.getMean());
        telemetry.update();
    }


}
