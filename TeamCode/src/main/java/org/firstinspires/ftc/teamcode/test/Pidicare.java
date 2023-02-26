package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;


@Config
@Autonomous(name = "pidicare", group = "B")
public class Pidicare extends LinearOpMode {
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

        robot.init();

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            robot.lift.setPower(update(POZITIE, robot.ridicare1.getCurrentPosition()));;

            dashboardTelemetry.addData("Target Position", POZITIE);
            dashboardTelemetry.addData("Current Position", robot.ridicare1.getCurrentPosition());
            dashboardTelemetry.addData("Power Motor", robot.ridicare1.getPower());
            dashboardTelemetry.update();
        }
    }




    public double update(double target, double state) {
        double error = target - state;
        double derivative = (error - lastError) / robot.ridicareTimer.seconds();
        integralSum += error * robot.ridicareTimer.seconds();
        if (integralSum > INTEGRALSUMILIMIT) {
            integralSum = INTEGRALSUMILIMIT;
        }
        if (integralSum < -INTEGRALSUMILIMIT) {
            integralSum = -INTEGRALSUMILIMIT;
        }
        double output = KP * error + KI * integralSum + KD * derivative;
        lastError = error;
        return output;



    }

}

