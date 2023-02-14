package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Medii", group = "A", preselectTeleOp = "TeleOP")
public class Medii extends OpMode {

    SampleMecanumDrive drive;
    Pose2d startPose;
    RobotHardware robot;



    Trajectory inceput, align, cone, mid;

    @Override
    public void init(){
        robot = new RobotHardware(this);
        startPose = new Pose2d(35, -60.355, Math.toRadians(270));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
         inceput = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(35, -35, Math.toRadians(315)))
                 .addDisplacementMarker(()->drive.followTrajectoryAsync(align))
                .build();
         align = drive.trajectoryBuilder(inceput.end())
                .lineToLinearHeading(new Pose2d(36,-12, 0))
                 .addDisplacementMarker( ()->drive.followTrajectoryAsync(cone))
                .build();
         cone = drive.trajectoryBuilder(align.end())
                 .addDisplacementMarker(()->drive.followTrajectoryAsync(mid))
                .forward(18)
                .build();
         mid = drive.trajectoryBuilder(cone.end())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(45)))
                 .addDisplacementMarker(1.7, () -> robot.lift.target = robot.RIDICARE_POS_2)
                 .addDisplacementMarker( () -> drive.followTrajectory(cone))
                .build();

    }
    @Override
    public void start(){
        robot.start();
        drive.followTrajectory(inceput);
    }
    public void loop() {
         drive.update();
         robot.lift.update();

    }
}
