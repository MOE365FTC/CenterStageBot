package org.firstinspires.ftc.teamcode.autonomus;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.backup.backupHardware.BackupMOEBot;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Autonomous
public class PowerPlayBlueRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        BackupMOEBot robot = new BackupMOEBot(hardwareMap, gamepad1, gamepad2);

        Pose2d startPose = new Pose2d(-62, -35, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .forward(29)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.backupClaw.release();
                })
                .waitSeconds(0.5)
                .back(25)
                .strafeLeft(24)
                .forward(24)
                .strafeLeft(28)
                .lineToLinearHeading(new Pose2d(-60, 50, Math.toRadians(90)))
                .build();


        while(!isStarted() && !isStopRequested()) {
            robot.backupClaw.grab();
        }

        waitForStart();

        drive.followTrajectorySequence(traj1);


    }
}
