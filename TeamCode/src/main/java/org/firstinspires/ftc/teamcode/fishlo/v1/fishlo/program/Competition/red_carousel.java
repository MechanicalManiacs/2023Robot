package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.TSEDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.stream.Stream;

@Autonomous
public class red_carousel extends FishloAutonomousProgram {

    TSEDetectionPipeline.BarcodePosition position;
    ElapsedTime timer;
    SampleMecanumDrive mdrive;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        timer = new ElapsedTime();
        telemetry.setAutoClear(true);
        telemetry.addLine("Start");
        telemetry.update();
        mdrive = new SampleMecanumDrive(hardwareMap);
        vision.initVision();
        while (!isStarted()) {
            position = vision.getPlacement();
            telemetry.addData("Position", position);
            telemetry.update();
        }

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
        sleep(7000);

        //drive.driveleft(13.5, 0.5, false, 0);

        //sleep(200);
        mdrive.setPoseEstimate(new Pose2d());
        Pose2d start_pose = new Pose2d(0,0, Math.toRadians(0));
//
//        position = TSEDetectionPipeline.BarcodePosition.CENTER;

        switch (position) {
            case LEFT:
                intake.armToLevel(0, false, 0);
                break;
            case CENTER:
                intake.armToLevel(1, false, 0);
                break;
            case RIGHT:
                intake.armToLevel(2, false, 0);
                break;
            case NULL:
                int random = (int)(Math.random()*3);
                intake.armToLevel(random, false, 0);
                telemetry.addData("Default Postion", random);
                telemetry.update();
        }

        telemetry.addLine("2. Squaring up with wall");
        telemetry.update();
        telemetry.addLine("3. Move arm to position");
        telemetry.update();
        Trajectory to_hub  = mdrive.trajectoryBuilder(start_pose)
                .splineToConstantHeading(new Vector2d(25,  -16),Math.toRadians(0))
                .build();
        mdrive.followTrajectory(to_hub);
        sleep(200);
        mdrive.turn(Math.toRadians(-10));
        Intake.intake.setPower(-.6);
        sleep(1000);
        intake.intake(Intake.IntakeState.OFF);

        mdrive.turn(Math.toRadians(15));

        Trajectory to_wall = mdrive.trajectoryBuilder(to_hub.end(), true)
                .splineToConstantHeading(new Vector2d(0 ,20), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, 30), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        mdrive.followTrajectory(to_wall);
        Trajectory back = mdrive.trajectoryBuilder(to_wall.end())
                .back(2)
                .build();
        mdrive.followTrajectory(back);

        sleep(100);
        sleep(100);
        intake.duck.setPower(-0.4);
        sleep(5000);
        Trajectory Park = mdrive.trajectoryBuilder(to_wall.end())
                .splineToConstantHeading(new Vector2d(22,38), Math.toRadians(0))
                .build();
        mdrive.followTrajectory(Park);













    }
}

