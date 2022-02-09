package org.firstinspires.ftc.teamcode.drive.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;
import com.spartronics4915.lib.T265Localizer;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.KalmanFilter;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class KalmanFilter265Localizer implements Localizer {
    T265Localizer localizer;
    KalmanFilter filter;
    public KalmanFilter265Localizer(T265Camera cam, HardwareMap hwmap) {
         localizer = new T265Localizer(cam);
         filter = new KalmanFilter(0, 0);
         filter.filterSetup();
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        filter.setym(localizer.getPoseEstimate().getY());
        filter.setxm(localizer.getPoseEstimate().getX());
        filter.runFilter();
        Pose2d pose2d = new Pose2d(filter.getStateUpdateX(), filter.getStateUpdateY(), localizer.getPoseEstimate().getHeading());
        return pose2d;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        localizer.setPoseEstimate(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        Pose2d pose2d = localizer.getPoseVelocity();
        return pose2d;
    }

    @Override
    public void update() {
        filter.setxm(localizer.getPoseEstimate().getX() * 47.2441196/39.37007874015748D);
        filter.setym(localizer.getPoseEstimate().getY() * 47.2441196/39.37007874015748D);
        filter.runFilter();
        localizer.setPoseEstimate(new Pose2d(filter.getStateUpdateX(), filter.getStateUpdateY(), localizer.getPoseEstimate().getHeading()));
    }

    public double getHeading() {
        return localizer.getPoseEstimate().getHeading();
    }
}
