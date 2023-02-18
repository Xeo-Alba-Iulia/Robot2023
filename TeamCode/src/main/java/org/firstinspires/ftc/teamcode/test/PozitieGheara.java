package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Disabled
@Config
@Autonomous
public class PozitieGheara extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    Servo gheara;
    public static double pozitie = 0.525;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        gheara = hardwareMap.get(Servo.class, "ServoGheara");
        gheara.setPosition(0.525);

        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            gheara.setPosition(pozitie);
            dashboardTelemetry.addData("pozitie", gheara.getPosition());
            dashboardTelemetry.update();
        }
    }

}
