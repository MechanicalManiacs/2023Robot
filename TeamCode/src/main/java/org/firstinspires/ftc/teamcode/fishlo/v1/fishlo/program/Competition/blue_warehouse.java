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
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Vector;

@Autonomous
public class blue_warehouse extends FishloAutonomousProgram {
    String position;
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
        telemetry.addLine("Dectecting Position of Barcode");
        while (!isStarted()) {
            if (vision.getPlacement().equals("Left") || vision.getPlacement().equals("Right") || vision.getPlacement().equals("Center")) {
                position = vision.getPlacement();
            }
            telemetry.addData("Position", position);
        }
        telemetry.update();

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
        mdrive = new SampleMecanumDrive(hardwareMap);
        //sleep(200);
        mdrive.setPoseEstimate(new Pose2d());
        Pose2d start_pose = new Pose2d(0, 0, Math.toRadians(0));

        switch (position) {
            case "Left":
                intake.armToLevel(0, false, 0);
                break;
            case "Center":
                intake.armToLevel(1, false, 0);
                break;
            case "Right":
                intake.armToLevel(2, false, 0);
                break;
        }

        Trajectory to_hub = mdrive.trajectoryBuilder(start_pose)
                .splineToConstantHeading(new Vector2d(20, -16), Math.toRadians(0))
                .build();
        mdrive.followTrajectory(to_hub);
//        Trajectory turn10 = mdrive.trajectoryBuilder(to_hub.end())
//                .splineToConstantHeading(new Vector2d(to_hub.end().getX(), to_hub.end().getY()), Math.toRadians(10))
//                .build();
//        mdrive.followTrajectory(turn10);
//        intake.intake(Intake.IntakeState.REVERSE);
//        sleep(500);
//        intake.intake(Intake.IntakeState.OFF);
//        sleep(100);
//
//        Trajectory turn90 = mdrive.trajectoryBuilder(turn10.end())
//                .splineToConstantHeading(new Vector2d(turn10.end().getX(), turn10.end().getY()), Math.toRadians(-90))
//                .build();
//        mdrive.followTrajectory(turn90);



        sleep(200);
        mdrive.turn(Math.toRadians(10));
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(500);
        intake.intake(Intake.IntakeState.OFF);
        //drops the bonus block
        sleep(100);
        mdrive.turn(Math.toRadians(-85));
        intake.stop();
        Pose2d turn_pose = new Pose2d(to_hub.end().getX(), to_hub.end().getY(), Math.toRadians(-90));
        Trajectory Test = mdrive.trajectoryBuilder(turn_pose)
                .splineToConstantHeading(new Vector2d(-10, 5), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    intake.intake(Intake.IntakeState.ON);

                })
                .splineToConstantHeading(new Vector2d(-10,35), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-10, 45), Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        //drive to the warehouse
        mdrive.followTrajectory(Test);



        Trajectory back_test = mdrive.trajectoryBuilder(turn_pose)
                .splineToConstantHeading(new Vector2d(0, 5), Math.toRadians(-90))

                .build();

        mdrive.followTrajectory(back_test);
        intake.intake(Intake.IntakeState.OFF);
        Trajectory bt2 = mdrive.trajectoryBuilder(back_test.end(), true)
                .splineToConstantHeading(new Vector2d(18, 14), Math.toRadians(-90))
                .build();

        mdrive.followTrajectory(bt2);
        intake.armToLevel(4, true, 1500);
        mdrive.turn(Math.toRadians(-50));
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(967);
        intake.armToLevel(0,true,0);
        mdrive.turn(Math.toRadians(-50));

        sleep(500);
        Pose2d allah = new Pose2d(to_hub.end().getX(), to_hub.end().getY(), Math.toRadians(-90));
        Trajectory pls_work = mdrive.trajectoryBuilder(allah)
                .splineToConstantHeading(new Vector2d(-15, 5), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    intake.intake(Intake.IntakeState.ON);

                })
                .splineToConstantHeading(new Vector2d(-10,-5), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-15, 45), Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        mdrive.followTrajectory(pls_work);










//
//        sleep(100);
//
//
//        Trajectory line_wall = mdrive.trajectoryBuilder(to_hub.end())
//                .splineToConstantHeading(new Vector2d(2, -5), Math.toRadians(0))
//                .build();
//        mdrive.followTrajectory(line_wall);
//        Trajectory Back = mdrive.trajectoryBuilder(line_wall.end())
//                .back(5)
//                .build();
//        mdrive.followTrajectory(Back);
//        intake.intake(Intake.IntakeState.ON);
//        Trajectory get_block = mdrive.trajectoryBuilder(line_wall.end())
//                .lineToConstantHeading(new Vector2d(3, -30))
//                .build();
//        mdrive.followTrajectory(get_block);
//        sleep(500);
//        intake.intake.setPower(0);
//        sleep(200);
//        intake.intake(Intake.IntakeState.OFF);
//        Trajectory return_block = mdrive.trajectoryBuilder(get_block.end())
//                .lineToConstantHeading(new Vector2d(3, 0))
//                .build();
//        mdrive.followTrajectory(return_block);



    }


}


