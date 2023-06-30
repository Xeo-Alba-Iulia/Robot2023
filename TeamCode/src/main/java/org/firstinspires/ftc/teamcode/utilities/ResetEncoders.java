package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.sisteme.Ridicare;

@TeleOp
public class ResetEncoders extends LinearOpMode {
    DcMotorEx ridicare1;
    DcMotorEx ridicare2;
    Ridicare lift;

    @Override
    public void runOpMode() {
        ridicare1 = hardwareMap.get(DcMotorEx.class, "RidicareAproape");
        ridicare2 = hardwareMap.get(DcMotorEx.class, "RidicareDeparte");
        lift = new Ridicare(ridicare1, ridicare2);
        telemetry.addData("Poz Ridicare", lift.getCurrentPosition());
        waitForStart();
        lift.resetEncoders();

    }
}
