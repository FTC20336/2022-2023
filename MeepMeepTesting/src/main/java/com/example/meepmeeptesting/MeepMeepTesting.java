package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        double startDir = Math.toRadians(90);

        /*
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16, 16)
                .setConstraints(52.48180821614297, 52.48180821614297, 4.434444427490234, Math.toRadians(184.02607784577722), 13.52)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(90)))
                                /*
                                .strafeRight(24)
                                .forward(12)
                                .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(90))
                                .strafeLeft(12)
                                .forward(20)
                                .lineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(180)))
                                .forward(4)
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))


                                .splineToConstantHeading(new Vector2d(-13, -54), startDir)
                                .lineTo(new Vector2d(-13, -42))
                                .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))
                                .waitSeconds(1)
                                .setTangent(48)
                                .splineToConstantHeading(new Vector2d(-12, -32), startDir)
                                .splineTo(new Vector2d(-12, -12), startDir)
                                .setTangent(22)
                                .splineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(180)), Math.toRadians(180))
                                .lineTo(new Vector2d(-60, -12))
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))

                                /*
                                .lineTo(new Vector2d(-12, -12))
                                .lineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(180)))
                                .lineTo(new Vector2d(-60, -12))
                                .lineToSplineHeading(new Pose2d(-24, -8, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-60, -12, Math.toRadians(180)))


                                .lineToSplineHeading(new Pose2d(-36, -30, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-36, -64, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-36, -30, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-36, -64, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-36, -30, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-36, -64, Math.toRadians(90)))
                                .build()
                );

         */


        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16, 15)
                .setConstraints(52.48180821614297, 52.48180821614297, 4.434444427490234, Math.toRadians(184.02607784577722), 13.52)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40, -64.5, Math.toRadians(90)))
                                .setTangent(Math.toRadians(10))
                                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(90))
                                .lineToConstantHeading( new Vector2d(-12, -40))
                                .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90))
                                .lineToConstantHeading(new Vector2d(0, -30))
                                .lineToConstantHeading(new Vector2d(0, -36))
                                .lineToConstantHeading(new Vector2d(-12, -36))
                                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-60, -12))
                                .lineToSplineHeading(new Pose2d(-24, -7.5, Math.toRadians(90)))
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