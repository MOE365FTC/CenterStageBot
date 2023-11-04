package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.List;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Image bgIMG = null;
        try{ bgIMG = ImageIO.read(new File("../CenterStageBot/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/CenterStageTopDown.jpg")); }
        catch(IOException e) {}

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-62, -35, 0))

                                //COORDINATE SYSTEM SOMEHOW WORKS NOW CAUSE IMAGE IS ROTATED, MAYBE WILL CAUSE ISSUE
                                //blueDown; Pose2d(-62, -35, 0)
                                //vision incomplete; vedic
                                .splineTo(new Vector2d(-37,-35),Math.toRadians(0))
                                .turn(Math.toRadians(-90))
                                .waitSeconds(0.25)
                                //spike deposit incomplete; mechanical
                                .splineToSplineHeading(new Pose2d(-37, -55, Math.toRadians(90)), Math.toRadians(0))
                                .lineTo(new Vector2d(-11.5,-54))
//                                .lineTo(new Vector2d(-12, 17))
//                                .splineTo(new Vector2d(-32,48), Math.toRadians(90))
//                                .waitSeconds(0.25)
//                                //pixel backdrop output incomplete; mechanical
//                                .splineTo(new Vector2d(-12, 17), Math.toRadians(-90))
//                                .lineTo(new Vector2d(-11.5,-60))
//                                //pixel backdrop output incomplete; mechanical
//                                .lineTo(new Vector2d(-12, 17))
//                                .splineTo(new Vector2d(-32,48), Math.toRadians(90))
//                                .strafeLeft(14)
//                                .back(10)
                                .build()










//                                //redDown; Pose2d(62, -35, Math.toRadians(180))
//                                //vision incomplete; vedic
//                                .splineTo(new Vector2d(33,-35),Math.toRadians(180))
//                                .waitSeconds(0.25)
//                                //spike deposit incomplete; mechanical
//                                .splineTo(new Vector2d(11.5,-60),Math.toRadians(-90))
//                                .lineTo(new Vector2d(12, 17))
//                                .splineTo(new Vector2d(32,48), Math.toRadians(90))
//                                .waitSeconds(0.25)
//                                //pixel backdrop output incomplete; mechanical
//                                .lineTo(new Vector2d(36, -60))
//                                .lineTo(new Vector2d(36, 17))
//                                .splineTo(new Vector2d(32,48), Math.toRadians(90))
//                                .waitSeconds(0.25)
//                                //pixel backdrop output incomplete; mechanical
//                                .lineTo(new Vector2d(36, -60))
//                                .lineTo(new Vector2d(36, 17))
//                                .splineTo(new Vector2d(32,48), Math.toRadians(90))
//                                .waitSeconds(0.25)
//                                //pixel backdrop output incomplete; mechanical
//                                .strafeLeft(28)
//                                .back(12)
//                                .build()







                                /*
                                //blueUp; Pose2d(-62, 12, Math.toRadians(0))
                                //vision incomplete; vedic
                                .splineTo(new Vector2d(-33,12),Math.toRadians(0))
                                .waitSeconds(0.25)
                                //spike deposit incomplete; mechanical
                                .splineTo(new Vector2d(-32,46), Math.toRadians(90))
                                .waitSeconds(0.25)
                                //pixel backdrop output incomplete; mechanical
                                .lineTo(new Vector2d(-36, -60))
                                .lineTo(new Vector2d(-36, 17))
                                .splineTo(new Vector2d(-32,48), Math.toRadians(90))
                                .waitSeconds(0.25)
                                //pixel backdrop output incomplete; mechanical
                                .lineTo(new Vector2d(-36, -60))
                                .lineTo(new Vector2d(-36, 17))
                                .splineTo(new Vector2d(-32,48), Math.toRadians(90))
                                .waitSeconds(0.25)
                                //pixel backdrop output incomplete; mechanical
                                .strafeLeft(28)
                                .forward(10)
                                .build()
                                //possible more cycles...

                                 */









                                /*
                                //redUp; Pose2d(62, 12, Math.toRadians(180))
                                //vision incomplete; vedic
                                .splineTo(new Vector2d(33,12),Math.toRadians(180))
                                .waitSeconds(0.25)
                                //spike deposit incomplete; mechanical
                                .splineTo(new Vector2d(32,46), Math.toRadians(90))
                                .waitSeconds(0.25)
                                //pixel backdrop output incomplete; mechanical
                                .lineTo(new Vector2d(36, -60))
                                .lineTo(new Vector2d(36, 17))
                                .splineTo(new Vector2d(32,48), Math.toRadians(90))
                                .waitSeconds(0.25)
                                //pixel backdrop output incomplete; mechanical
                                .lineTo(new Vector2d(36, -60))
                                .lineTo(new Vector2d(36, 17))
                                .splineTo(new Vector2d(32,48), Math.toRadians(90))
                                .waitSeconds(0.25)
                                //pixel backdrop output incomplete; mechanical
                                .strafeRight(28)
                                .forward(10)
                                .build()

                                 */



                );

        meepMeep.setBackground(bgIMG)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}