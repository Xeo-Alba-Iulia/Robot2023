package org.firstinspires.ftc.teamcode.opmodes.teleop.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "parcare simpla", group = "A", preselectTeleOp = "prototype")
public class ParcareSimpla extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();
    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        waitForStart();
        if(isStopRequested()) {
            return;
        }
        time.reset();
        while(time.seconds() < 1.2) {
            robot.frontRight.setPower(0.5);
            robot.frontLeft.setPower(0.5);
            robot.backLeft.setPower(0.5);
            robot.backRight.setPower(0.5);
        }
    }

}
