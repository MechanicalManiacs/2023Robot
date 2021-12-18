package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.acmerobotics.roadrunner.geometry.Vector2d;
        import com.acmerobotics.roadrunner.trajectory.Trajectory;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
        import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
        import org.firstinspires.ftc.teamcode.robot.Robot;

        import java.util.ArrayList;

@Autonomous
public class ThreadBuilder extends FishloAutonomousProgram {

    public static SampleMecanumDrive mecanumDrive;
    public static Pose2d startPose;

    // Target Zone A Trajectories
    static Trajectory targetZoneATraj1;
    static Trajectory targetZoneATraj2;
    static Trajectory targetZoneATraj3;
    static Trajectory targetZoneATraj4;
    static Trajectory targetZoneATraj5;
    static Trajectory targetZoneATraj6;
//
//    // Target Zone B Trajectories
//    static Trajectory targetZoneBTraj1;
//    static Trajectory targetZoneBTraj2;
//    static Trajectory targetZoneBTraj3;
//    static Trajectory targetZoneBTraj4;
//    static Trajectory targetZoneBTraj5;
//    static Trajectory targetZoneBTraj6;
//
//    // Target Zone C Trajectories
//    static Trajectory targetZoneCTraj1;
//    static Trajectory targetZoneCTraj2;
//    static Trajectory targetZoneCTraj3;
//    static Trajectory targetZoneCTraj4;
//    static Trajectory targetZoneCTraj5;
//    static Trajectory targetZoneCTraj6;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    // edited start position(rahul)
    @Override
    public void preMain() {
        startPose = new Pose2d(-36, -72, Math.toRadians(180));
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(startPose);
        telemetry.addLine("Ready to Start |>");
    }

    @Override
    public void main() {
        // Make TrajectoryBuilderA thread
        TrajectoryBuilderA trajectoryBuilderA = new TrajectoryBuilderA();
        trajectoryBuilderA.setRobot(getRobot());

        // Make TrajectoryBuilderB thread
//        TrajectoryBuilderB trajectoryBuilderB = new TrajectoryBuilderB();
//        trajectoryBuilderB.setRobot(getRobot());

        // Make TrajectoryBuilderC thread
//        TrajectoryBuilderC trajectoryBuilderC = new TrajectoryBuilderC();
//        trajectoryBuilderC.setRobot(getRobot());

        //Start all of the threads



        try {
            trajectoryBuilderA.start();

            telemetry.addLine("Thread A started");
            telemetry.update();
            telemetry.addLine("Joining thread A");
            telemetry.update();
            trajectoryBuilderA.join();
//            trajectoryBuilderB.start();

//            telemetry.addLine("Thread B started");
//            telemetry.update();
//            telemetry.addLine("Joining thread B");
//            telemetry.update();
//            trajectoryBuilderB.join();
//
//            telemetry.addLine("Thread C started");
//            telemetry.update();
//            trajectoryBuilderC.start();
//            telemetry.addLine("Joining thread C");
//            telemetry.update();
//            trajectoryBuilderC.join();

        } catch (Exception e) {
            telemetry.addLine(e.toString());
            telemetry.update();
        }



        // Get all of the target zone A trajectories
        ArrayList<Trajectory> targetZoneATrajList = trajectoryBuilderA.getTrajectoryList();

        targetZoneATraj1 = targetZoneATrajList.get(0);
        targetZoneATraj2 = targetZoneATrajList.get(1);
        targetZoneATraj3 = targetZoneATrajList.get(2);
        targetZoneATraj4 = targetZoneATrajList.get(3);
        targetZoneATraj5 = targetZoneATrajList.get(4);
        targetZoneATraj6 = targetZoneATrajList.get(5);

        telemetry.addLine("Target Zone A Trajectories Completed");
        telemetry.update();
//
//        // Get all of the target zone B trajectories
//        ArrayList<Trajectory> targetZoneBTrajList = trajectoryBuilderB.getTrajectoryList();
//
//        targetZoneBTraj1 = targetZoneBTrajList.get(0);
//        targetZoneBTraj2 = targetZoneBTrajList.get(1);
//        targetZoneBTraj3 = targetZoneBTrajList.get(2);
//        targetZoneBTraj4 = targetZoneBTrajList.get(3);
//        targetZoneBTraj5 = targetZoneBTrajList.get(4);
//        targetZoneBTraj6 = targetZoneBTrajList.get(5);
//
//        telemetry.addLine("Target Zone B Trajectories Completed");
//        telemetry.update();
//
//        // Get all of the target zone C trajectories
//        ArrayList<Trajectory> targetZoneCTrajList = trajectoryBuilderC.getTrajectoryList();
//
//        targetZoneCTraj1 = targetZoneCTrajList.get(0);
//        targetZoneCTraj2 = targetZoneCTrajList.get(1);
//        targetZoneCTraj3 = targetZoneCTrajList.get(2);
//        targetZoneCTraj4 = targetZoneCTrajList.get(3);
//        targetZoneCTraj5 = targetZoneCTrajList.get(4);
//        targetZoneCTraj6 = targetZoneCTrajList.get(5);
//        telemetry.addLine("Target Zone C Trajectories Completed");
//        telemetry.update();

//        sleep(5000);
    }


}

// Thread to build target zone A trajectories
class TrajectoryBuilderA extends Thread {
    private volatile ArrayList<Trajectory> trajectoryList = new ArrayList<Trajectory>();
    private SampleMecanumDrive mecanumDrive;
    private Pose2d startPose;
    private Robot robot;

    // Run method
    public void run() {
        try {
            // Build trajectories (rahul edited position)
            Trajectory targetZoneATraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
                    .splineToConstantHeading(new Vector2d(-36, -54), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-36, -36), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneATraj1);

            Trajectory targetZoneATraj2 = mecanumDrive.trajectoryBuilder(targetZoneATraj1.end())
                    .strafeRight(10)
                    .build();
            trajectoryList.add(targetZoneATraj2);

            Trajectory targetZoneATraj3 = mecanumDrive.trajectoryBuilder(targetZoneATraj2.end())
                    .splineToConstantHeading(new Vector2d(5, -5), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-59, -4), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneATraj3);


            Trajectory targetZoneATraj4 = mecanumDrive.trajectoryBuilder(targetZoneATraj3.end(), true)
                    .strafeLeft(13)
                    .build();
            trajectoryList.add(targetZoneATraj4);

            Trajectory targetZoneATraj5 = mecanumDrive.trajectoryBuilder(targetZoneATraj4.end(), true)
                    .splineToConstantHeading(new Vector2d(-24, -16), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(12, -47), Math.toRadians(0))
                    .build();
            trajectoryList.add(targetZoneATraj5);


            Trajectory targetZoneATraj6 = mecanumDrive.trajectoryBuilder(targetZoneATraj5.end(), true)
                    .back(3)
                    .build();
            trajectoryList.add(targetZoneATraj6);

            robot.telemetry.addLine("Target Zone A Trajectories Built");
            robot.telemetry.update();


        } catch (Exception e) {
            // Throwing an exception
            robot.telemetry.addLine("Exception is caught");
        }
    }


    public TrajectoryBuilderA() {
        mecanumDrive = ThreadBuilder.mecanumDrive;
        startPose = ThreadBuilder.startPose;
    }

    // Set the robot
    public void setRobot(Robot r) {
        robot = r;
    }

    // Get the trajectory list
    public ArrayList<Trajectory> getTrajectoryList() {
        return trajectoryList;
    }
}

// Thread to build target zone B trajectories
//class TrajectoryBuilderB extends Thread {
//    private volatile ArrayList<Trajectory> trajectoryList = new ArrayList<Trajectory>();
//    private SampleMecanumDrive mecanumDrive;
//        try
//        {
//            // Displaying the thread that is running
//            Trajectory targetZoneBTraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
//                    .splineToConstantHeading(new Vector2d(-20, -55), Math.toRadians(0))
//                    .splineToConstantHeading(new Vector2d(47, -27), Math.toRadians(0))
//                    .build();
//            trajectoryList.add(targetZoneBTraj1);
//
//
//            Trajectory targetZoneBTraj2 = mecanumDrive.trajectoryBuilder(targetZoneBTraj1.end())
//                    .splineToConstantHeading(new Vector2d(17, -27), Math.toRadians(0))
//                    .splineToConstantHeading(new Vector2d(19, -52), Math.toRadians(0))
//                    .build();
//            trajectoryList.add(targetZoneBTraj2);
//
//
//            Trajectory targetZoneBTraj3 = mecanumDrive.trajectoryBuilder(targetZoneBTraj2.end())
//                    .splineToConstantHeading(new Vector2d(10, -47), Math.toRadians(0))
//                    .splineToConstantHeading(new Vector2d(-10, -47), Math.toRadians(0))
//                    .build();
//            trajectoryList.add(targetZoneBTraj3);
//
//
//            Trajectory targetZoneBTraj4 = mecanumDrive.trajectoryBuilder(targetZoneBTraj3.end(), true)
//                    .splineToConstantHeading(new Vector2d(10, -47), Math.toRadians(0))
//                    .splineToConstantHeading(new Vector2d(19, -52), Math.toRadians(0))
//                    .build();
//            trajectoryList.add(targetZoneBTraj4);
//
//
//            Trajectory targetZoneBTraj5 = mecanumDrive.trajectoryBuilder(targetZoneBTraj4.end(), true)
//                    .splineToConstantHeading(new Vector2d(0, -5), Math.toRadians(0))
//                    .splineToConstantHeading(new Vector2d(60, -25), Math.toRadians(0))
//                    .build();
//            trajectoryList.add(targetZoneBTraj5);
//
//
//            Trajectory targetZoneBTraj6 = mecanumDrive.trajectoryBuilder(targetZoneBTraj5.end())
//                    .forward(8)
//                    .build();
//            trajectoryList.add(targetZoneBTraj6);
//
//            robot.telemetry.addLine("Target Zone B Trajectories Built");
//            robot.telemetry.update();
//        }
//        catch (Exception e)
//        {
//            // Throwing an exception
//            robot.telemetry.addLine("Exception is caught");
//        }
//    }

//    public TrajectoryBuilderB() {
//        mecanumDrive = ThreadBuilder.mecanumDrive;
//        startPose = ThreadBuilder.startPose;
//    }
//
//    public void setRobot(Robot r) {
//        robot = r;
//    }
//    public ArrayList<Trajectory> getTrajectoryList() {
//        return trajectoryList;
//    }
//}

// Thread to build target zone C trajectories
//class TrajectoryBuilderC extends Thread {
//    private volatile ArrayList<Trajectory> trajectoryList = new ArrayList<Trajectory>();
//    private SampleMecanumDrive mecanumDrive;
//    private Pose2d startPose;
//    private Robot robot;
//
//    public void run() {
//        try
//        {
//             Displaying the thread that is running
//            Trajectory targetZoneCTraj1 = mecanumDrive.trajectoryBuilder(startPose, true)
//                    .splineToConstantHeading(new Vector2d(-20, -49), Math.toRadians(0))
//                    .splineToConstantHeading(new Vector2d(68, -40), Math.toRadians(0))
//                    .build();
//            trajectoryList.add(targetZoneCTraj1);
//
//            Trajectory targetZoneCTraj2 = mecanumDrive.trajectoryBuilder(targetZoneCTraj1.end())
//                    .splineToConstantHeading(new Vector2d(17, -40), Math.toRadians(0))
//                    .splineToConstantHeading(new Vector2d(14, -41), Math.toRadians(0))
//                    .build();
//            trajectoryList.add(targetZoneCTraj2);
//
//            Trajectory targetZoneCTraj3 = mecanumDrive.trajectoryBuilder(targetZoneCTraj2.end())
//                    .splineToConstantHeading(new Vector2d(5, -5), Math.toRadians(0))
//                    .splineToConstantHeading(new Vector2d(-49.75, -6), Math.toRadians(0))
//                    .build();
//            trajectoryList.add(targetZoneCTraj3);
//
//            Trajectory targetZoneCTraj4 = mecanumDrive.trajectoryBuilder(targetZoneCTraj3.end(), true)
//                    .strafeLeft(7.5)
//                    .build();
//            trajectoryList.add(targetZoneCTraj4);
//
//            Trajectory targetZoneCTraj5 = mecanumDrive.trajectoryBuilder(targetZoneCTraj4.end())
//                    .splineToConstantHeading(new Vector2d(-24, -16), Math.toRadians(0))
//                    .splineToConstantHeading(new Vector2d(70, -47), Math.toRadians(0))
//                    .build();
//            trajectoryList.add(targetZoneCTraj5);
//
//            Trajectory targetZoneCTraj6 = mecanumDrive.trajectoryBuilder(targetZoneCTraj5.end())
//                    .forward(20)
//                    .build();
//            trajectoryList.add(targetZoneCTraj6);
//
//            robot.telemetry.addLine("Target Zone C Trajectories Built");
//            robot.telemetry.update();
//
//
//
//        }
//        catch (Exception e)
//        {
//             Throwing an exception
//            robot.telemetry.addLine("Exception is caught");
//        }
//    }
//
//    public TrajectoryBuilderC() {
//        mecanumDrive = ThreadBuilder.mecanumDrive;
//        startPose = ThreadBuilder.startPose;
//    }
//
//    public void setRobot(Robot r) {
//        robot = r;
//    }
//
//    public ArrayList<Trajectory> getTrajectoryList() {
//        return trajectoryList;
//    }