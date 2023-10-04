package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

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
                        drive.trajectorySequenceBuilder(new Pose2d(62, 12, Math.toRadians(180)))
                                /*
                                //blueDown; Pose2d(-62, -35, 0)
                                //vision incomplete; vedic
                                .splineTo(new Vector2d(-33,-35),Math.toRadians(0))
                                .waitSeconds(0.5)
                                //spike deposit incomplete; mechanical
                                .splineTo(new Vector2d(-11.5,-60),Math.toRadians(-90))
                                .lineToSplineHeading(new Pose2d(-12, 17, Math.toRadians(90)))
                                .splineTo(new Vector2d(-32,48), Math.toRadians(90))
                                //pixel backdrop output incomplete; mechanical
                                .splineToConstantHeading(new Vector2d(-18.5, 40), Math.toRadians(45))
                                .splineToConstantHeading(new Vector2d(-14,61),Math.toRadians(180))
                                .build()
                                //possible more cycles...
                                 */



                                /*
                                //redDown; Pose2d(62, -35, Math.toRadians(180))
                                //vision incomplete; vedic
                                .splineTo(new Vector2d(33,-35),Math.toRadians(180))
                                .waitSeconds(0.5)
                                //spike deposit incomplete; mechanical
                                .splineTo(new Vector2d(11.5,-60),Math.toRadians(-90))
                                .lineToSplineHeading(new Pose2d(12, 17, Math.toRadians(90)))
                                .splineTo(new Vector2d(32,48), Math.toRadians(90))
                                //pixel backdrop output incomplete; mechanical
                                .splineToConstantHeading(new Vector2d(18.5, 40), Math.toRadians(135))
                                .splineToConstantHeading(new Vector2d(14,61),Math.toRadians(0))
                                .build()
                                //possible more cycles...
                                */



                                /*
                                //blueUp; Pose2d(-62, 12, Math.toRadians(0))
                                //vision incomplete; vedic
                                .splineTo(new Vector2d(-33,12),Math.toRadians(0))
                                .waitSeconds(0.25)
                                //spike deposit incomplete; mechanical
                                .splineTo(new Vector2d(-32,46), Math.toRadians(90))
                                .waitSeconds(0.25)
                                //pixel backdrop output incomplete; mechanical
                                .splineToConstantHeading(new Vector2d(-55,45),Math.toRadians(135))
                                .splineToConstantHeading(new Vector2d(-59,59),Math.toRadians(90))
                                .build()
                                //possible more cycles...
                                 */



                                /*
                                //redUp; Pose2d(62, 12, Math.toRadians(180))
                                //vision incomplete; vedic
                                .splineTo(new Vector2d(33,12),Math.toRadians(180))
                                .waitSeconds(0.25)
                                //spike deposit incomplete; mechanical
                                .lineToSplineHeading(new Pose2d(34,46, Math.toRadians(90)))
                                .waitSeconds(0.25)
                                //pixel backdrop output incomplete; mechanical
                                .splineToConstantHeading(new Vector2d(55,45),Math.toRadians(45))
                                .splineToConstantHeading(new Vector2d(59,59),Math.toRadians(90))
                                .build()
                                //possible more cycles...
                                */
                );

        meepMeep.setBackground(bgIMG)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}