package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TrackingWheelForwardOffsetTuner;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.TSEDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

import java.util.Vector;

public class RedWare extends FishloAutonomousProgram {
    TSEDetectionPipeline.BarcodePosition position;
    ElapsedTime timer;
    SampleMecanumDrive mdrive;
    Trajectory to_hub;
    Trajectory ware;
    Trajectory getBlock;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        mdrive = new SampleMecanumDrive(hardwareMap);
        mdrive.setPoseEstimate(new Pose2d());
        Pose2d start_pose = new Pose2d(0, 0, Math.toRadians(0));
        timer = new ElapsedTime();
        telemetry.setAutoClear(true);
        drive.initGyro();
        telemetry.addLine("Dectecting Position of Barcode");
        while (!isStarted()) {
            position = vision.getPlacement();
            telemetry.addData("Position", position);
            telemetry.update();
        }
        to_hub = mdrive.trajectoryBuilder(start_pose)
                .splineToConstantHeading(new Vector2d(20, 24), Math.toRadians(0))
                .build();
        telemetry.addLine("to hub built");
        Pose2d hubPose = new Pose2d(0, 0, Math.toRadians(0));
        ware = mdrive.trajectoryBuilder(hubPose)
                .splineTo(new Vector2d(7,-29), Math.toRadians(-180))
                .build();
        telemetry.addLine("ware built");
        Pose2d blockPose = new Pose2d(0, 0, Math.toRadians(0));
        getBlock = mdrive.trajectoryBuilder(blockPose)
                .addDisplacementMarker(() -> {
                    intake.intake(Intake.IntakeState.ON);

                })
                .splineTo(new Vector2d(0,-39), Math.toRadians(-180))
                .build();
        telemetry.addLine("get block built");
    }

    // strafe right is positive power, strafe left is negative power!
//measurements subject to change
    //Re-push cuz rahul is bad
    @Override
    public void main() {
        vision.stop();
        telemetry.clear();
        telemetry.update();
        telemetry.addLine("1. Strafing");
        telemetry.update();

        //sleep(200);


//        switch (position) {
//            case LEFT:
//                intake.armToLevel(0, false, 0);
//                break;
//            case CENTER:
//                intake.armToLevel(1, false, 0);
//                break;
//            case RIGHT:
//                intake.armToLevel(2, false, 0);
//                break;
//            case NULL:
//                int random = (int)(Math.random()*3);
//                intake.armToLevel(random, false, 0);
//                telemetry.addData("Default Postion", random);
//                telemetry.update();
        intake.armToLevel(2, false, 0);
        mdrive.followTrajectory(to_hub);
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(500);
        intake.intake(Intake.IntakeState.OFF);
        mdrive.followTrajectory(ware);

        mdrive.followTrajectory(getBlock);

        }















    }



