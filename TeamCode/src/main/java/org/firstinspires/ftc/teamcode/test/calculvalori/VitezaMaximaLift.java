package org.firstinspires.ftc.teamcode.test.calculvalori;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.utilities.PIDController;

@Autonomous(name = "VitezaMaxima", group = "calcule")
public class VitezaMaximaLift extends LinearOpMode {

    DcMotorEx ridicare1;
    DcMotorEx ridicare2;
    Ridicare lift;
    Telemetry dashboardTelemetry;
    ElapsedTime timer = new ElapsedTime();
    PIDController ridicareController = new PIDController(0.00296, 0, 0.01865059, this);

    double maxSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ridicare1 = hardwareMap.get(DcMotorEx.class, "RidicareAproape");
        ridicare2 = hardwareMap.get(DcMotorEx.class, "RidicareDeparte");
        lift = new Ridicare(ridicare1, ridicare2);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        lift.resetEncoder();


        waitForStart();
        if (isStopRequested()) {
            return;
        }
        timer.reset();

        while (opModeIsActive()) {
            if (timer.seconds() < 5) {
                dashboardTelemetry.addData("Start in ", 5-timer.seconds());
                dashboardTelemetry.addData("Current Speed", 0);
                dashboardTelemetry.addData("Max Speed", 0);
                dashboardTelemetry.update();
            } else {
                lift.setPower(ridicareController.update(18000, lift.getCurrentPosition()));
                double currentSpeed = lift.getAverageVelocity();
                if (maxSpeed <= currentSpeed) {
                    maxSpeed = currentSpeed;
                }
                dashboardTelemetry.addData("Current Speed", currentSpeed);
                dashboardTelemetry.addData("Max Speed", maxSpeed);
                dashboardTelemetry.update();
            }
        }
    }
}
