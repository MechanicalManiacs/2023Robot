package org.firstinspires.ftc.teamcode.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.KalmanFilter;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.T265Manager;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class KalmanFilter265Localizer implements Localizer {

    private static Pose2d mPosEstimate = new Pose2d();
    private Pose2d rawPose = new Pose2d();
    private Pose2d poseOffset = new Pose2d();

    private T265Camera.CameraUpdate up;

    private KalmanFilter filter;

    public static T265Camera slamra;

    private static T265Camera.PoseConfidence confidence;

    public KalmanFilter265Localizer(HardwareMap hwmap) {
        new KalmanFilter265Localizer(hwmap, true);
    }

    public KalmanFilter265Localizer(HardwareMap hwmap, boolean resetPos) {
        mPosEstimate = new Pose2d();
        rawPose = new Pose2d();
        poseOffset = new Pose2d();

        if (slamra == null) {
            T265Manager.get(hwmap);
            RobotLog.d("Created T265 object");
            setPoseEstimate(new Pose2d(0, 0, 0));
        }
        if (filter == null) {
            filter = new KalmanFilter(0, 0);
            filter.filterSetup();
        }
        try {
            slamra.start();
        }
        catch (Exception ignored) {
            RobotLog.v("T265 already started");
            if (resetPos) {
                slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0,0, new Rotation2d(0)));
                filter.setxm(0);
                filter.setym(0);
            }
        }
        if (slamra.getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) {
            RobotLog.e("T265 failed to get position");
        }
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        if (up != null) {
            Translation2d oldPose = up.pose.getTranslation();
            Rotation2d oldRotation = up.pose.getRotation();
            filter.setxm(oldPose.getX() * 39.3701);
            filter.setym(oldPose.getY() * 39.3701);
            filter.runFilter();
            rawPose = new Pose2d(filter.getStateUpdateX() / 0.0254, filter.getStateUpdateY() / 0.0245, norm(oldRotation.getRadians()));
            mPosEstimate = rawPose.plus(poseOffset);
        }
        else {
            RobotLog.v("NULL Camera update");
        }
        return mPosEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        RobotLog.v("Set Pose to: " + pose2d.toString());
        pose2d = new Pose2d(pose2d.getX(), pose2d.getY(), 0);
        RobotLog.v("SETTING POSE ESTIMATE TO: " + pose2d.toString());
        poseOffset = pose2d.minus(rawPose);
        poseOffset = new Pose2d(poseOffset.getX(), poseOffset.getY(), Math.toRadians(0));
        RobotLog.v("SET POSE OFFSET TO: " + poseOffset.toString());
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        ChassisSpeeds velo = up.velocity;
        return new Pose2d(velo.vxMetersPerSecond / 0.0254, velo.vyMetersPerSecond / 0.0254, velo.omegaRadiansPerSecond);
    }

    @Override
    public void update() {
        up = slamra.getLastReceivedCameraUpdate();
        confidence = up.confidence;
    }

    private double norm(double angle) {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }

    private static double norma(double angle) {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }

    public static T265Camera.PoseConfidence getConfidence() {
        return confidence;
    }

    public static double getHeading() {
        return norma(mPosEstimate.getHeading());
    }
}
