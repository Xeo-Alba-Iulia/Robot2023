package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utilities.PIDController;


@Config
@Autonomous(name = "pidicare", group = "B")
public class Pidicare extends LinearOpMode {

    public static int POZITIE = 0;
    public static double Kp = 0 , Ki = 0, Kd = 0;
    double lastKp, lastKi, lastKd;
    RobotHardware robot = new RobotHardware(this);
    //    public static double MARGIN;
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init();

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            if(lastKp != Kp || lastKi != Ki || lastKd != Ki) {
                robot.lift.controller = new PIDController(Kp, Ki, Kd, this);
            }

            robot.lift.target = POZITIE;
            robot.lift.update();
            lastKp = Kp;
            lastKi = Ki;
            lastKd = Kd;
            dashboardTelemetry.addData("Target Position", POZITIE);
            dashboardTelemetry.addData("Current Position", -robot.lift.getCurrentPosition());
            dashboardTelemetry.addData( "Power Motor", robot.lift.getPower());
            dashboardTelemetry.addData("Timer", robot.lift.controller.timer.seconds());
            dashboardTelemetry.update();
        }
    }

}

