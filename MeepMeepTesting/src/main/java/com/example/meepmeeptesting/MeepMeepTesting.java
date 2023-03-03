package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(850);
        Pose2d startPose = new Pose2d(36, -65.5, Math.toRadians(270));
        TrajectorySequence secventa;




        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(62.8429523186, 62.8429523186, 4.676666736602783, 4.676666736602783, 8)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .back(20)
//                                .lineToLinearHeading(new Pose2d(35,-21,Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(35,-12, Math.toRadians(0)))
//                                .lineTo(new Vector2d(57, -12))
//                                .lineTo(new Vector2d(38,-13))
//                                .lineToLinearHeading(new Pose2d(33,-15,Math.toRadians(25)))
//                                .lineToLinearHeading(new Pose2d(13,-15, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(35,-12, Math.toRadians(0)))
//                                .lineTo(new Vector2d(57,-12))



                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .setAxesInterval(5)
                .start();
    }
}