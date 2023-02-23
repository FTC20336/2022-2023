package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        double startDir = Math.toRadians(90);

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16, 15)
                .setConstraints(61*.5, 61*.5, 4.434444427490234*.5, Math.toRadians(184.02607784577722), 13.52)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40, -64.5, Math.toRadians(90)))
                                .setTangent(Math.toRadians(10))
                                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(90))
                                .splineToConstantHeading( new Vector2d(-12, -46), Math.toRadians(90))
                                .splineToSplineHeading( new Pose2d(-12, -24, Math.toRadians(0)), Math.toRadians(90))
                                .forward(6)
                                .setTangent(Math.toRadians(140))
                                .splineToSplineHeading(new Pose2d(-24,-11, Math.toRadians(180)),  Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-70+19,-12),Math.toRadians(180))
                                .forward(12) // PIck up Cone
                                .splineToConstantHeading(new Vector2d(-58,-10),Math.toRadians(180))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(-56,-14, Math.toRadians(315)), Math.toRadians(315))
                                .forward(6) //Drop COne
                                .setTangent(Math.toRadians(135))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-57,-11,Math.toRadians(180)), Math.toRadians(180))
                                .forward(6) // Pickup COne
                                .splineToConstantHeading(new Vector2d(-57,-12),Math.toRadians(180))
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-31,-13,Math.toRadians(315)), Math.toRadians(315))
                                .forward(6) //Drop COne
                                .setReversed(true)
                                .setTangent(Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(-30,-12, Math.toRadians(270)))
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-12,-12), Math.toRadians(0))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                //.addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }
}