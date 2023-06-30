package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;


@Config
@Autonomous(name = "Forwardicare", group = "B")
public class Forwardicare extends LinearOpMode {

    public static double Kv, Kp, Kg = 0.1;
    public static int target = 0;
    public static int toleranta = 120;
    public static int referenceSpeed = 2400;
    double lastKp, lastKi, lastKd, lastKg;
    double lastTarget;
    int sign=1;
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init();
        robot.lift.resetEncoders();

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {


            int currentPosition = robot.lift.getCurrentPosition();
            double currentSpeed = robot.ridicare1.getVelocity();


            boolean targetReached = currentPosition < (target + toleranta) && currentPosition > (target - toleranta);
            dashboardTelemetry.addData("Conditie", targetReached);
            if(target!= lastTarget)
            {
                if(target<lastTarget)
                    sign = -1;
                else
                    sign = 1;
            }
            if (!targetReached)
            {
                robot.lift.setVelocity(sign * (Kp * (referenceSpeed - currentSpeed) + Kv * referenceSpeed) );
                dashboardTelemetry.addLine("letsgo");
            } else
                robot.lift.setVelocity(0);
            dashboardTelemetry.addData("pozitie", robot.lift.getCurrentPosition());
            robot.lift.printVelocity(dashboardTelemetry);
            dashboardTelemetry.update();
            lastTarget = target;

        }


    }

}



