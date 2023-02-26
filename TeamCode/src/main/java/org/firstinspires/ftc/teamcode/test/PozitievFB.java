package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;


@Config
@Autonomous
public class PozitievFB extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    Servo vFB1;
    Servo vFB2;
    public static double pozitievFB1 = 0;
    public static double pozitieVFB2 = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        vFB1 = hardwareMap.get(Servo.class, "vfb1");
        vFB2= hardwareMap.get(Servo.class, "vfb2");

        vFB1.setPosition(0);
        vFB2.setPosition(0);

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            vFB1.setPosition(pozitievFB1);
            vFB2.setPosition(pozitieVFB2);
            dashboardTelemetry.addData("pozitie vFB1", vFB1.getPosition());
            dashboardTelemetry.addData("pozitie vFB2", vFB2.getPosition());

            dashboardTelemetry.update();
        }
    }

}
