package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class red_warehouse extends FishloAutonomousProgram {
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
                .splineToConstantHeading(new Vector2d(20, 16), Math.toRadians(0))
                .build();
        mdrive.followTrajectory(to_hub);

        sleep(200);
        mdrive.turn(Math.toRadians(10));
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(500);
        intake.intake(Intake.IntakeState.OFF);
        sleep(100);

        sleep(100);


        Trajectory line_wall = mdrive.trajectoryBuilder(to_hub.end())
                .splineToConstantHeading(new Vector2d(2, -5), Math.toRadians(0))
                .build();
        mdrive.followTrajectory(line_wall);
        Trajectory Back = mdrive.trajectoryBuilder(line_wall.end())
                .back(5)
                .build();
        mdrive.followTrajectory(Back);
        intake.intake(Intake.IntakeState.ON);
        Trajectory get_block = mdrive.trajectoryBuilder(line_wall.end())
                .lineToConstantHeading(new Vector2d(3, -30))
                .build();
        mdrive.followTrajectory(get_block);
        sleep(500);
        intake.intake.setPower(0);
        sleep(200);
        intake.intake(Intake.IntakeState.OFF);
        Trajectory return_block = mdrive.trajectoryBuilder(get_block.end())
                .lineToConstantHeading(new Vector2d(3, 0))
                .build();
        mdrive.followTrajectory(return_block);



    }


}

