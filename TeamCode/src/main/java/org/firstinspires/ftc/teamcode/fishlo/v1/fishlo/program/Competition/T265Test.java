package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.KalmanFilter;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous
public class T265Test extends FishloAutonomousProgram {

    KalmanFilter filter;
    T265Camera apache;
    ElapsedTime timer = new ElapsedTime();
    boolean free = false;


    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        filter = new KalmanFilter(0, 0);
        filter.filterSetup();
        telemetry.setAutoClear(true);
        apache = T265Helper.getCamera(new T265Camera.OdometryInfo(), hardwareMap.appContext);
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
        apache.setPose(new Pose2d(0, 0, Math.toRadians(0)));
        Timer t = new Timer();
        t.schedule(new TimerTask() {
            @Override
            public void run() {
                filter.setxm(apache.getLastReceivedCameraUpdate().pose.getY() * 47.2441196/39.37007874015748D);
                filter.setym(apache.getLastReceivedCameraUpdate().pose.getX() * 47.2441196/39.37007874015748D);
                filter.runFilter();
                telemetry.addData("PosXFilter", filter.getStateUpdateX());
                telemetry.addData("PosYFilter", filter.getStateUpdateY());
                telemetry.addData("KalmanGain", filter.getGain());
                telemetry.addData("PosYCam", apache.getLastReceivedCameraUpdate().pose.getX() /** 47.2441196*/);
                telemetry.addData("PosXCam", apache.getLastReceivedCameraUpdate().pose.getY() /** 47.2441196*/);
                telemetry.addData("DegCam", apache.getLastReceivedCameraUpdate().pose.getHeading());
                telemetry.addData("RadCam", Math.toRadians(apache.getLastReceivedCameraUpdate().pose.getHeading()));
                telemetry.update();
                if (isStopRequested()) {
                    apache.stop();
                    free = true;
                    t.cancel();
                }
            }
        }, 0, 100);
        while (!isStopRequested()) {
            sleep(2000);
        }
    }
}
