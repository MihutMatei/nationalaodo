package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(250), Math.toRadians(250), 11.81)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11, 59, Math.toRadians(-90)))

                                .splineToLinearHeading(new Pose2d(-5, 44, Math.toRadians(70)), Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(14, 62, Math.toRadians(0)), Math.toRadians(60))
                                .lineToSplineHeading(new Pose2d(40, 62, Math.toRadians(0)))
                                .lineTo(
                                        new Vector2d(60, 62),
                                        SampleMecanumDrive.getVelocityConstraint(10,90, 90),
                                        SampleMecanumDrive.getAccelerationConstraint(10)
                                )
                                .lineToSplineHeading(new Pose2d(14, 62, Math.toRadians(0)))

                                //luat cub 1
                                .splineToLinearHeading(new Pose2d(-5, 44, Math.toRadians(70)), Math.toRadians(-60))
                                .splineToLinearHeading(new Pose2d(14, 62, Math.toRadians(0)), Math.toRadians(60))
                                .lineToSplineHeading(new Pose2d(40, 62, Math.toRadians(0)))
                                .lineTo(
                                        new Vector2d(60, 62),
                                        SampleMecanumDrive.getVelocityConstraint(10,90, 90),
                                        SampleMecanumDrive.getAccelerationConstraint(10)
                                )
                                .lineToSplineHeading(new Pose2d(14, 62, Math.toRadians(0)))
                                //luat cub 2
                                .splineToLinearHeading(new Pose2d(-5, 44, Math.toRadians(70)), Math.toRadians(-60))
                                .splineToLinearHeading(new Pose2d(14, 62, Math.toRadians(0)), Math.toRadians(60))
                                .lineToSplineHeading(new Pose2d(40, 62, Math.toRadians(0)))
                                .lineTo(
                                        new Vector2d(60, 62),
                                        SampleMecanumDrive.getVelocityConstraint(10,90, 90),
                                        SampleMecanumDrive.getAccelerationConstraint(10)
                                )
                                .lineToSplineHeading(new Pose2d(14, 62, Math.toRadians(0)))
                                //luat cub 3
                                .splineToLinearHeading(new Pose2d(-5, 44, Math.toRadians(70)), Math.toRadians(-60))
                                .splineToLinearHeading(new Pose2d(14, 62, Math.toRadians(0)), Math.toRadians(60))
                                .lineToSplineHeading(new Pose2d(50, 62, Math.toRadians(0)))

                                //luat cub 4
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

