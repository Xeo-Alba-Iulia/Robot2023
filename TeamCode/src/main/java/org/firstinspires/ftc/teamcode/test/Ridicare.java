package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Disabled
@Config
@Autonomous(name = "pidicare", group = "C")
public class Ridicare extends LinearOpMode {
    DcMotor motor = null;

    public static double KP = 0, KI = 0, KD = 0;
    public static double INTEGRALSUMILIMIT = 0.25;
    public static double POZITIE = 0;
    RobotHardware robot = new RobotHardware(this);
    double integralSum = 0;
    double lastError = 0;
//    public static double MARGIN;
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        boolean aWasPressed = false;

        robot.init();

        waitForStart();

        if(isStopRequested()) return;



        while(opModeIsActive()) {
           if(gamepad1.a) {
               aWasPressed = true;
           } else if(aWasPressed) {
                robot.ridicare.setPower(0);
                aWasPressed = false;
            } else {
                robot.ridicare.setPower(update(POZITIE, robot.ridicare.getCurrentPosition()));
            }
            dashboardTelemetry.addData("Target Position", POZITIE);
            dashboardTelemetry.addData("Current Position", robot.ridicare.getCurrentPosition());
            dashboardTelemetry.update();
        }
    }



    public double update(double target, double state) {
        state = -state;
        double error = 1.0 - target / state;
        double derivative = (error - lastError) / robot.timer.seconds();
        integralSum += error * robot.timer.seconds();
        double output = KP * error + KI * integralSum + KD * derivative;
        lastError = error;
        return output;
    }

}
