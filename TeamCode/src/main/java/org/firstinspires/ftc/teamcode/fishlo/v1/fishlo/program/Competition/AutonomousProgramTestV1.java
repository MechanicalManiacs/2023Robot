package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.OpenCV;
import org.firstinspires.ftc.teamcode.robot.Robot;
@Autonomous
public class DropNShootRoadRunnerAuto extends FishloAutonomousProgram {

    //Back right corner is (-72, -72)
    //Back left corner is (-72, 24)
    //Front right corner is (72, -72)
    //Front left corner is (72, 24)

    //Global Variables
    // yesss i am cool kid forever wakanda elon musk

    public static SampleMecanumDrive mecanumDrive;
    public static Pose2d startPose;
    public static boolean autoEnded = false;
    public volatile static Pose2d endPose;
    protected OpenCV.targetZone targetZone;

    // Target Zone A Trajectories
    private Trajectory targetZoneATraj1;
    private Trajectory targetZoneATraj2;
    private Trajectory targetZoneATraj3;
    private Trajectory targetZoneATraj4;
    private Trajectory targetZoneATraj5;
    private Trajectory targetZoneATraj6;

    // Target Zone B Trajectories
    private Trajectory targetZoneBTraj1;
    private Trajectory targetZoneBTraj2;
    private Trajectory targetZoneBTraj3;
    private Trajectory targetZoneBTraj4;
    private Trajectory targetZoneBTraj5;
    private Trajectory targetZoneBTraj6;

    // Target Zone C Trajectories
    private Trajectory targetZoneCTraj1;
    private Trajectory targetZoneCTraj2;
    private Trajectory targetZoneCTraj3;
    private Trajectory targetZoneCTraj4;
    private Trajectory targetZoneCTraj5;
    private Trajectory targetZoneCTraj6;

    // Pose Tracker Thread
    PoseTracker poseTracker;

    // Gives the robot object so we can access it's methods
    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    // Init
    @Override
    public void preMain() {


        autoEnded = false;
        // Make the SampleMecanumDrive object for RoadRunner
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-63, -40, Math.toRadians(180));
        // Set the start pose
        mecanumDrive.setPoseEstimate(startPose);

        // Start the pose tracker thread
        poseTracker = new PoseTracker(mecanumDrive, startPose);
        poseTracker.start();

        openCV.initVision();

        targetZoneATraj1 = ThreadBuilder.targetZoneATraj1;
        targetZoneATraj2 = ThreadBuilder.targetZoneATraj2;
        targetZoneATraj3 = ThreadBuilder.targetZoneATraj3;
        targetZoneATraj4 = ThreadBuilder.targetZoneATraj4;
        targetZoneATraj5 = ThreadBuilder.targetZoneATraj5;
        targetZoneATraj6 = ThreadBuilder.targetZoneATraj6;

        targetZoneBTraj1 = ThreadBuilder.targetZoneBTraj1;
        targetZoneBTraj2 = ThreadBuilder.targetZoneBTraj2;
        targetZoneBTraj3 = ThreadBuilder.targetZoneBTraj3;
        targetZoneBTraj4 = ThreadBuilder.targetZoneBTraj4;
        targetZoneBTraj5 = ThreadBuilder.targetZoneBTraj5;

        targetZoneCTraj1 = ThreadBuilder.targetZoneCTraj1;
        targetZoneCTraj2 = ThreadBuilder.targetZoneCTraj2;
        targetZoneCTraj3 = ThreadBuilder.targetZoneCTraj3;
        targetZoneCTraj4 = ThreadBuilder.targetZoneCTraj4;
        targetZoneCTraj5 = ThreadBuilder.targetZoneCTraj5;




        telemetry.addLine("Ready for start");
        telemetry.update();

        // Get the target zone from the webcam
        while (!isStarted()) {

            targetZone = openCV.getTargetZone();
            telemetry.clear();
            telemetry.addData("Target Zone: ", targetZone);
            telemetry.addData("Rings: ", openCV.getHeight());
            telemetry.update();
        }
    }

    @Override
    public void main() {

        autoEnded = false;
        // Show the target zone
        telemetry.addData("TargetZone", targetZone);

        /**
         * Follow the appropriate trajectories
         */

        telemetry.addLine("Starting Program");
        telemetry.update();
        if (targetZone == OpenCV.targetZone.A) {

            // Move to Target Zone A and drop wobble goal
            telemetry.addLine("Moving to target zone A");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj1);

            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(1000);
            claw.armUp();

            // Move to shooting position
            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj2);

            claw.close();
            claw.armDown();
            sleep(500);
            // Shoot
            telemetry.addLine("Shooting");
            telemetry.update();
            shoot();
            claw.armUp();
            sleep(500);
            claw.open();


            // Move to second wobble goal
            telemetry.addLine("Moving to second wobble goal");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj3);

            // Grab wobble goal

            telemetry.addLine("Grabbing wobble goal");
            telemetry.update();
            claw.armDown();;
            mecanumDrive.followTrajectory(targetZoneATraj4);
            claw.close();
            sleep(1000);
            claw.armUp();

            // Move back to target zone A and drop wobble goal
            telemetry.addLine("Moving back to target zone A");
            mecanumDrive.followTrajectory(targetZoneATraj5);


            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();


            // Park
            telemetry.addLine("Parking");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneATraj6);

        }
        if (targetZone == OpenCV.targetZone.B) {

            // Move to target zone B and drop wobble goal
            telemetry.addLine("Moving to target zone B");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj1);

            // Drop wobble goal
            telemetry.addLine("Dropping wobble goal");
            telemetry.update();
            claw.armDown();
            sleep(200);
            claw.open();
            sleep(500);
            claw.armUp();

            // Move to shooting position
            telemetry.addLine("Moving to shooting position");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj2);

            intake.intakeRelease();
            claw.close();
            claw.armDown();
            sleep(500);
            // Shoot rings
            telemetry.addLine("Shooting");
            telemetry.update();
            shoot();
            claw.armUp();
            claw.open();
            sleep(500);
//
            intake.startIntake();
            // Move to second wobble goal
            telemetry.addLine("Getting rings");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj3);

            sleep(1500);

            // Grab wobble goal
            telemetry.addLine("Going back to shoot");
            telemetry.update();
            mecanumDrive.followTrajectory(targetZoneBTraj4);
            intake.stopIntake();

            claw.close();
            claw.armDown();
            sleep(500);
            // Shoot rings
            telemetry.addLine("Shooting");
            telemetry.update();
            shoot();
            claw.armUp();
            claw.open();
            sleep(500);

        }

        poseTracker.stopThread();
        try {
            poseTracker.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        autoEnded = true;

    }


    // Rotate function
    public Vector2d rotate(Vector2d point) {
        double x = point.getX();
        double y = point.getY();
        Vector2d rotatedPoint = new Vector2d(y, -x);
        return rotatedPoint;
    }
}



class PoseTracker extends Thread {
    private SampleMecanumDrive mecanumDrive;
    private boolean exit;

    public PoseTracker(SampleMecanumDrive drive, Pose2d startPose) {
        mecanumDrive = drive;
        mecanumDrive.setPoseEstimate(startPose);
        exit = false;
    }

    public void run() {
        while (!exit) {
            DropNShootRoadRunnerAuto.endPose = mecanumDrive.getPoseEstimate();
        }
    }

    public void stopThread() {
        exit = true;
    }
}