package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Drive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Fishlo;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Vision;
import org.firstinspires.ftc.teamcode.opMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Rahul extends FishloAutonomousProgram {

    SampleMecanumDrive drive;
    Pose2d position;

    @Override
    protected Robot buildRobot() {return super.buildRobot();
    }

    @Override
    public void preMain() {
        drive =  new SampleMecanumDrive(hardwareMap);
        position = new Pose2d(0,1,Math.toRadians(30));
    }
    @Override
    public void main() {
        Trajectory myTrajectory = drive.trajectoryBuilder(position).forward(5).build();

    }


}