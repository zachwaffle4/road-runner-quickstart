package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.roadrunner.Pose2d;
import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class CommandBuilder extends NextFTCOpMode {
    private final Pose2d startPose = new Pose2d(9.0, 111.0, Math.toRadians(-90.0));
    private final Pose2d scorePose = new Pose2d(16.0, 128.0, Math.toRadians(-45.0));
    
    MecanumDrive drive;
    Command driveCommand;

    @Override
    public void onInit() {
        drive = new MecanumDrive(hardwareMap, startPose);

        driveCommand = drive.commandBuilder(startPose)
                .splineTo(scorePose.position, scorePose.heading)
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        driveCommand.schedule();
    }
}
