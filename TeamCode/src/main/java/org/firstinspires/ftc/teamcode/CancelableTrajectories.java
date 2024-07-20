package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class CancelableTrajectories {
    public static TrajectorySequence getTrajectory(
            SampleMecanumDriveCancelable drive,
            int targetPos
    ) {
        Pose2d end;

        switch (targetPos) {
            case 0: end = new Pose2d(51.00, -29.50, Math.toRadians(180.00)); break;
            case 2: end = new Pose2d(51.00, -42.50, Math.toRadians(180.00)); break;
            default: end = new Pose2d(51.00, -36.00, Math.toRadians(180.00));
        }

        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(end, Math.toRadians(0.00))
                .build();
    }
}
