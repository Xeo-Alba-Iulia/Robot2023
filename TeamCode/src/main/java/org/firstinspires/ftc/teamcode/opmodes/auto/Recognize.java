package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.camera.XeoCamera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
public class Recognize extends OpMode {

    XeoCamera camera = new XeoCamera();
    SampleMecanumDrive drive;
    int randomization;
    Pose2d outtakePose = new Pose2d(36,-12, 0);
    Trajectory park;

    @Override
    public void init() {
        randomization = 0;
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(outtakePose);



    }

    @Override
    public void init_loop() {
        if(randomization==0) {
            randomization = camera.getCase();
        }

    }

    @Override
    public void loop() {


    }
}
