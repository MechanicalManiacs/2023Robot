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

@Autonomous
public class WareRed extends FishloAutonomousProgram {
    TSEDetectionPipeline.BarcodePosition position;
    ElapsedTime timer;
    SampleMecanumDrive mdrive;

    Trajectory trajectory;
    Pose2d endTraj;


    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        timer = new ElapsedTime();
        telemetry.setAutoClear(true);
        drive.initGyro();
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
        //pre-stuff
        telemetry.addLine("I hate ftc");
        telemetry.update();
        vision.stop();
        telemetry.clear();
        mdrive = new SampleMecanumDrive(hardwareMap);
        mdrive.setPoseEstimate(new Pose2d());
        Pose2d start_pose = new Pose2d(0, 0, Math.toRadians(0));

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
        telemetry.addLine("Before trajectory");

        //to hub
        trajectory = mdrive.trajectoryBuilder(start_pose)
                .splineToConstantHeading(new Vector2d(19, 25), Math.toRadians(0))
                .build();
        telemetry.addLine("After");
        mdrive.followTrajectory(trajectory);
        endTraj = trajectory.end();
        //drop block
        Intake.intake.setPower(-.65);
        sleep(500);
        intake.intake(Intake.IntakeState.OFF);
        //nudge backward
        trajectory = mdrive.trajectoryBuilder(endTraj)
                .lineTo(new Vector2d(11,25))
                .build();
        mdrive.followTrajectory(trajectory);
        endTraj = trajectory.end();
        intake.stop();
        intake.resetEncoder();
        telemetry.addLine("Before turn");
        telemetry.update();
        mdrive.turn(Math.toRadians(-78));
        telemetry.addLine("finished turning");
        telemetry.update();
        intake.intake(Intake.IntakeState.ON);
        //line up with wall and go straight
        Pose2d lpose= new Pose2d(endTraj.getX(), endTraj.getY(), Math.toRadians(-90));
        trajectory = mdrive.trajectoryBuilder(lpose)
                .splineToConstantHeading(new Vector2d(-15,10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-15, -25), Math.toRadians(-90))
                .build();
        telemetry.addLine("i stg");
        telemetry.update();
        mdrive.followTrajectory(trajectory);
        endTraj = trajectory.end();
        telemetry.addLine("i stg part 2");
        telemetry.update();
        //grab block
        Pose2d turn_pose = new Pose2d(endTraj.getX(), endTraj.getY(), Math.toRadians(-90));
        trajectory = mdrive.trajectoryBuilder(turn_pose)
                .forward(3,
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        mdrive.followTrajectory(trajectory);
        endTraj = trajectory.end();
        telemetry.addLine("about to start new trajectory");
        telemetry.update();

        //comes out of warehouse
        trajectory = mdrive.trajectoryBuilder(endTraj)
            .splineToConstantHeading(new Vector2d(-15, 20), Math.toRadians(-90))
            .build();
        telemetry.addLine("Built Trajectory block_place");
        telemetry.update();
        mdrive.followTrajectory(trajectory);
        intake.intake(Intake.IntakeState.OFF);
        endTraj = trajectory.end();
        telemetry.addLine("Finished running Trajectory block_place");
        telemetry.update();
        intake.armToLevel(2, false, 0);
        //go back to hub
        trajectory = mdrive.trajectoryBuilder(endTraj)
                .splineToLinearHeading(new Pose2d(6, 24), Math.toRadians(0))
                .build();
        telemetry.addLine("built strafing");
        telemetry.update();
        mdrive.followTrajectory(trajectory);
        endTraj = trajectory.end();
        telemetry.addLine("Finished strafing");
        telemetry.update();
        //release block
        Intake.intake.setPower(-.25);
        sleep(1000);
        intake.intake(Intake.IntakeState.OFF);
        telemetry.addLine("about the start the process for the second block...");
        telemetry.update();
        //nudge backward
        trajectory = mdrive.trajectoryBuilder(endTraj)
                .lineTo(new Vector2d(-18,25))
                .build();
        mdrive.followTrajectory(trajectory);
        endTraj = trajectory.end();
        intake.stop();
        intake.resetEncoder();
        telemetry.addLine("Before turn");
        telemetry.update();
        mdrive.turn(Math.toRadians(-78));
        intake.stop();
        telemetry.addLine("finished turning");
        telemetry.update();
        //go to wall and straight into warehouse
        Pose2d wpose = new Pose2d(endTraj.getX(), endTraj.getY(), Math.toRadians(-90));
        trajectory = mdrive.trajectoryBuilder(wpose)
                .splineToConstantHeading(new Vector2d(-20,10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-20, -35), Math.toRadians(-90))
                .build();
        mdrive.followTrajectory(trajectory);
        endTraj = trajectory.end();
        intake.intake(Intake.IntakeState.ON);
        //grab block
        trajectory = mdrive.trajectoryBuilder(endTraj)
                .forward(4,
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        mdrive.followTrajectory(trajectory);
        sleep(1000);
        intake.intake(Intake.IntakeState.OFF);
//        Trajectory block_place1 = mdrive.trajectoryBuilder(mdrive.getPoseEstimate())
//                .splineToConstantHeading(new Vector2d(-15, 10), Math.toRadians(-90))
//                .build();
//        mdrive.followTrajectory(block_place1);
//        Trajectory strafe_left1 = mdrive.trajectoryBuilder(mdrive.getPoseEstimate())
////                    .strafeLeft(5)
//                .splineToLinearHeading(new Pose2d(5, 20), Math.toRadians(0))
//                .build();
//        telemetry.addLine("built strafing");
//        telemetry.update();
//        mdrive.followTrajectory(strafe_left);
//        telemetry.addLine("Finished strafing");
//        telemetry.update();
//        intake.intake(Intake.IntakeState.REVERSE);
//        sleep(500);
//        intake.intake(Intake.IntakeState.OFF);







    }


}


