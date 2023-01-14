package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startDir = Math.toRadians(90);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16, 15)
                .setConstraints(52.48180821614297, 52.48180821614297, 4.434444427490234, Math.toRadians(184.02607784577722), 13.52)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(90)))
                                .setTangent(Math.toRadians(10))
                                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(-12, -40), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(0, -28), Math.toRadians(90))
                                .build()
                );


        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16, 15)
                .setConstraints(52.48180821614297, 52.48180821614297, 4.434444427490234, Math.toRadians(184.02607784577722), 13.52)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -65, Math.toRadians(90)))
                                .setTangent(Math.toRadians(10))
                                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(90))
                                .lineToConstantHeading( new Vector2d(-12, -40))
                                .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(0, -28))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }
}