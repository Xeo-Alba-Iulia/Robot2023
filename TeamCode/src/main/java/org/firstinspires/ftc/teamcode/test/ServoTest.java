package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sisteme.VirtualFourBar;

@Config
@TeleOp
public class ServoTest extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    private ServoImplEx servo1;
    private ServoImplEx servo2;

    public static double position = 0.35;
    ElapsedTime timer = new ElapsedTime();

    private static double pos = 0;

    VirtualFourBar virtualFourBar;


    @Override
    public void init() {
        servo1 =hardwareMap.get(ServoImplEx.class,"vfb1");
        servo2 =hardwareMap.get(ServoImplEx.class,"vfb2");
        virtualFourBar = new VirtualFourBar(servo1, servo2);
        virtualFourBar.setPosition(0.35);
    }

    @Override

    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {

        virtualFourBar.setPosition(position);

        telemetry.addData("poz1", servo1.getPosition());
        telemetry.addData("poz2", servo2.getPosition());

    }
}
