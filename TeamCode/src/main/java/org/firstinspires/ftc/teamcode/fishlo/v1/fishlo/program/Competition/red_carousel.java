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

@Autonomous
public class red_carousel extends FishloAutonomousProgram {

    String position;
    ElapsedTime timer;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        position = "";
        timer = new ElapsedTime();
        telemetry.setAutoClear(true);
        drive.initGyro();
        telemetry.addLine("Dectecting Position of Barcode");
        String placement = "";
        while (!isStarted()) {
            placement = vision.getPlacement();
            if (placement.equals("Left") || placement.equals("Right") || placement.equals("Center")) {
                position = placement;
                telemetry.addData("Position", position);
            }
            else if (placement.equals(null)) {
                telemetry.addLine("not found");
            }
        }
        telemetry.update();

    }
    // strafe right is positive power, strafe left is negative power!
//measurements subject to change
    //Re-push cuz rahul is bad
    @Override
    public void main() {
        if (!position.equals("Left") || !position.equals("Right") || !position.equals("Center")) {
            position = "Right";
            telemetry.addLine("Reverted to default vision");
            telemetry.update();
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        vision.stop();
        telemetry.clear();
        telemetry.update();
        telemetry.addLine("1. Strafing");
        telemetry.update();
        sleep(7000);

        //drive.driveleft(13.5, 0.5, false, 0);

        //sleep(200);
        drive.setPoseEstimate(new Pose2d());
        Pose2d start_pose = new Pose2d(0,0, Math.toRadians(0));

        switch(position) {
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

        telemetry.addLine("2. Squaring up with wall");
        telemetry.update();
        telemetry.addLine("3. Move arm to position");
        telemetry.update();
        Trajectory to_hub  = drive.trajectoryBuilder(start_pose)
                .splineToConstantHeading(new Vector2d(21,  -16),Math.toRadians(0))
                .build();
        drive.followTrajectory(to_hub);

        sleep(200);
        drive.turn(Math.toRadians(-12));
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(2000);
        intake.intake(Intake.IntakeState.OFF);
        sleep(100);
        drive.turn(Math.toRadians(15));

        sleep(100);
        Trajectory to_wall = drive.trajectoryBuilder(to_hub.end(), true)
                .splineToConstantHeading(new Vector2d(0 ,20), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, 30), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        drive.followTrajectory(to_wall);
        sleep(100);
        intake.duck.setPower(-0.4);
        sleep(5000);
        Trajectory Park = drive.trajectoryBuilder(to_wall.end())
                .splineToConstantHeading(new Vector2d(20,40), Math.toRadians(0))
                .build();
        drive.followTrajectory(Park);













    }
}

