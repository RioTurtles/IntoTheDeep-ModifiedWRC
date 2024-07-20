package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    private static Pose2d currentPose = new Pose2d();

    public static Pose2d getCurrentPose() {return currentPose;}
    public static void setCurrentPose(Pose2d newPose) {currentPose = newPose;}
}
