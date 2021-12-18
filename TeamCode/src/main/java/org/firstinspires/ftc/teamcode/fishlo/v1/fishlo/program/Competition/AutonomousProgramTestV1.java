package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
@Autonomous
public class AutonomousProgramTestV1 extends FishloAutonomousProgram {

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
        startPose = new Pose2d(-36, -72 , Math.toRadians(180));
        // Set the start pose
        mecanumDrive.setPoseEstimate(startPose);

        // Start the pose tracker thread
        poseTracker = new PoseTracker(mecanumDrive, startPose);
        poseTracker.start();


        targetZoneATraj1 = ThreadBuilder.targetZoneATraj1;
        targetZoneATraj2 = ThreadBuilder.targetZoneATraj2;
        targetZoneATraj3 = ThreadBuilder.targetZoneATraj3;
        targetZoneATraj4 = ThreadBuilder.targetZoneATraj4;
        targetZoneATraj5 = ThreadBuilder.targetZoneATraj5;
        targetZoneATraj6 = ThreadBuilder.targetZoneATraj6;

//        targetZoneBTraj1 = ThreadBuilder.targetZoneBTraj1;
//        targetZoneBTraj2 = ThreadBuilder.targetZoneBTraj2;
//        targetZoneBTraj3 = ThreadBuilder.targetZoneBTraj3;
//        targetZoneBTraj4 = ThreadBuilder.targetZoneBTraj4;
//        targetZoneBTraj5 = ThreadBuilder.targetZoneBTraj5;
//
//        targetZoneCTraj1 = ThreadBuilder.targetZoneCTraj1;
//        targetZoneCTraj2 = ThreadBuilder.targetZoneCTraj2;
//        targetZoneCTraj3 = ThreadBuilder.targetZoneCTraj3;
//        targetZoneCTraj4 = ThreadBuilder.targetZoneCTraj4;
//        targetZoneCTraj5 = ThreadBuilder.targetZoneCTraj5;




        telemetry.addLine("Ready for start");
        telemetry.update();

        // Get the target zone from the webcam
        while (!isStarted()) {

            telemetry.clear();
            telemetry.update();
        }
    }

    @Override
    public void main() {
        autoEnded = false;
        // Show the target zone

        /**
         * Follow the appropriate trajectories
         */

        telemetry.addLine("Starting Program");
        telemetry.update();
        mecanumDrive.followTrajectory(targetZoneATraj1);
        telemetry.addLine("1 - done");
        telemetry.update();
        sleep(1000);
        mecanumDrive.followTrajectory(targetZoneATraj2);

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
            AutonomousProgramTestV1.endPose = mecanumDrive.getPoseEstimate();
        }
    }

    public void stopThread() {
        exit = true;
    }
}