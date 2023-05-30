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

            if(Kp != lastKp || Ki != lastKi || Kd != lastKd) {
                robot.lift.controller = new PIDController(Kp, Ki, Kd);
            }
            robot.lift.target = POZITIE;
            robot.lift.update();
            lastKp = Kp;
            lastKi = Ki;
            lastKd = Kd;
            dashboardTelemetry.addData("Current Pos", robot.lift.getCurrentPosition());
            dashboardTelemetry.addData("Target", POZITIE);
            dashboardTelemetry.addData("pozitie ridicare 1" ,robot.ridicare1.getCurrentPosition());
            dashboardTelemetry.addData("pozitie ridicare 2", robot.ridicare2.getCurrentPosition());
            dashboardTelemetry.update();
        }
    }

}

