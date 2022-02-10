package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.ftcresearch.tfod.tracking.ObjectTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Objects;

import kotlin.math.UMathKt;

@Autonomous
public class red_warehouse extends FishloAutonomousProgram {
    String position;
    ElapsedTime timer;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        timer = new ElapsedTime();
        telemetry.setAutoClear(true);
        drive.initGyro();
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vision.stop();
        telemetry.clear();
        telemetry.update();
        telemetry.addLine("1. Strafing");
        telemetry.update();

        //drive.driveleft(13.5, 0.5, false, 0);

        //sleep(200);
        drive.setPoseEstimate(new Pose2d());
        Pose2d start_pose = new Pose2d(0, 0, Math.toRadians(0));


        telemetry.addLine("2. Squaring up with wall");
        telemetry.update();
        telemetry.addLine("3. Move arm to position");
        telemetry.update();
        Trajectory to_hub = drive.trajectoryBuilder(start_pose)
                .splineToConstantHeading(new Vector2d(20, 16), Math.toRadians(0))
                .build();
        drive.followTrajectory(to_hub);

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
        sleep(200);
        drive.turn(Math.toRadians(10));
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(100);
        intake.intake(Intake.IntakeState.OFF);
        sleep(100);
        drive.turn(Math.toRadians(-100));

        sleep(100);


        Trajectory line_wall = drive.trajectoryBuilder(to_hub.end())
                .splineToConstantHeading(new Vector2d(2, -5), Math.toRadians(0))
                .build();
        drive.followTrajectory(line_wall);
        Trajectory Back = drive.trajectoryBuilder(line_wall.end())
                .back(5)
                .build();
        drive.followTrajectory(Back);
        intake.armToLevel(0, false, 0);
        sleep(500);
        drive.turn(Math.toRadians(-75));
        intake.armToLevel(3, false, 0);
        intake.stop();
        intake.intake(Intake.IntakeState.ON);
        Trajectory get_block = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(3, -30))
                .build();
        drive.followTrajectory(get_block);
        sleep(500);
        intake.intake(Intake.IntakeState.OFF);
        Trajectory return_block = drive.trajectoryBuilder(get_block.end())
                .lineToConstantHeading(new Vector2d(3, 0))
                .build();
        drive.followTrajectory(return_block);



    }


}

