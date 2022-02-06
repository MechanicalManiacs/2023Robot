package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.KalmanFilter265;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.T265Manager;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous
public class T265Test extends FishloAutonomousProgram {

    KalmanFilter265 filter;
    T265Camera apache;
    ElapsedTime timer = new ElapsedTime();
    boolean free = false;


    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        filter = new KalmanFilter265(0, 0);
        filter.filterSetup();
        telemetry.setAutoClear(true);
        apache = T265Manager.get(hardwareMap);
        while (!apache.isStarted()) {
            apache.start();
        }
        while (!isStarted()) {
            telemetry.addData("Confidence", apache.getLastReceivedCameraUpdate().confidence);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        apache.setPose(new Pose2d(0, 0, new Rotation2d(0)));
        Timer t = new Timer();
        t.schedule(new TimerTask() {
            @Override
            public void run() {
                filter.setxm(apache.getLastReceivedCameraUpdate().pose.getY() * 39.3701);
                filter.setym(apache.getLastReceivedCameraUpdate().pose.getX() * 39.3701);
                filter.runFilter();
                telemetry.addData("PosXFilter", filter.getStateUpdateX());
                telemetry.addData("PosYFilter", filter.getStateUpdateY());
                telemetry.addData("KalmanGain", filter.getGain());
                telemetry.addData("PosYCam", apache.getLastReceivedCameraUpdate().pose.getX() * 39.3701);
                telemetry.addData("PosXCam", apache.getLastReceivedCameraUpdate().pose.getY() * 39.3701);
                telemetry.addData("DegCam", apache.getLastReceivedCameraUpdate().pose.getRotation().getDegrees());
                telemetry.addData("RadCam", apache.getLastReceivedCameraUpdate().pose.getRotation().getRadians());
                telemetry.update();
                if (isStopRequested()) {
                    apache.stop();
                    free = true;
                    t.cancel();
                }
            }
        }, 0, 500);
        while (!isStopRequested()) {
            sleep(2000);
        }
    }
}
