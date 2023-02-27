package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;


@Config
@Autonomous
public class Pozitii extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    public static double VFB = 1;
    public static int RIDICARE = 0;
    public static double GHEARA = 0;
    public static double ALLIGNER = 0;




    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        robot.init();
        robot.virtualFourBar.setPosition(1);
        robot.claw.setPosition(0);
        robot.claw_alligner.setPosition(0);


        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            robot.lift.target = RIDICARE;
            robot.lift.update();
            robot.virtualFourBar.setPosition(VFB);
            robot.claw.setPosition(GHEARA);
            robot.claw_alligner.setPosition(ALLIGNER);
            dashboardTelemetry.addData("pozitie vFB1", robot.vFB1.getPosition());
            dashboardTelemetry.addData("pozitie vFB2", robot.vFB2.getPosition());
            dashboardTelemetry.addData("pozitie ridicare", robot.lift.getCurrentPosition());
            dashboardTelemetry.addData("pozitie", VFB);
            dashboardTelemetry.addData("pozitie gheara", robot.claw.getPosition());
            dashboardTelemetry.addData("pozitie aliniere gheara", robot.claw_alligner.getPosition());
            dashboardTelemetry.update();
        }
    }

}
