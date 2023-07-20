package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer.WHEEL_RADIUS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name = "mafutinwindows")
public class TunerSmecherie extends LinearOpMode {
    private SampleMecanumDrive drive;
    double heading = 0;
    Encoder leftEncoder;
    Encoder rightEncoder;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * 1 * ticks / TICKS_PER_REV;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive= new SampleMecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        Pose2d startPose = new Pose2d(0, 0 ,Math.toRadians(90));
        int NUM = 5;
        double leftEncoderSum = 0;
        double rightEncoderSum = 0;
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "MotorBackRight"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "MotorBackLeft"));

        double headingAccumulator = 0;
        telemetry.addLine("ce pizdas masii");
        telemetry.update();
        waitForStart();
        telemetry.addLine("am ajuns");
        telemetry.update();//pt commit
        drive.setPoseEstimate(startPose);

        for (int i = 0; i < NUM; i++) {
            telemetry.clear();
            telemetry.addLine("am intrat");
            telemetry.update();

            // it is important to handle heading wraparounds
            double lastHeading = 0;

            drive.turnAsync(Math.toRadians(360));

            while (!isStopRequested() && drive.isBusy()) {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.normDelta(heading - lastHeading);
                lastHeading = heading;
                leftEncoderSum += encoderTicksToInches(leftEncoder.getCurrentPosition());
                rightEncoderSum += encoderTicksToInches(rightEncoder.getCurrentPosition());

            }

            telemetry.addData("Acumulator", headingAccumulator);
            telemetry.addData("Suma Stanga", leftEncoderSum);
            telemetry.addData("Suma Dreapta", rightEncoderSum);
            telemetry.update();
            sleep(1000);
        }
        telemetry.addData("raza dreapta", rightEncoderSum / headingAccumulator);
        telemetry.addData("raza stanga", leftEncoderSum/headingAccumulator);
        telemetry.update();
    }


}
