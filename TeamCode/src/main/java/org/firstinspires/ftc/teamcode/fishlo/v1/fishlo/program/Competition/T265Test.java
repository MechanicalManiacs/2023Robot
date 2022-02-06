package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.T265Manager;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

@Autonomous
public class T265Test extends FishloAutonomousProgram {

    T265Camera apache;
    ElapsedTime timer = new ElapsedTime();
    boolean free = false;


    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.setAutoClear(true);
        apache = T265Manager.get(hardwareMap);
        while (!apache.isStarted()) {
            apache.start();
        }
        while (!isStarted()) {
            telemetry.addData("Confidence", apache.getLastReceivedCameraUpdate().confidence);
            telemetry.update();
        }
//        rad = apache.getLastReceivedCameraUpdate().pose.getRotation().getRadians();
//        while (!isStarted()) {
//            if (rad > 0) {
//                rad -= 0.01;
//                telemetry.addData("Rad", rad);
//                telemetry.update();
//            } else if (rad < 0) {
//                rad += 0.01;
//                telemetry.addData("Rad", rad);
//                telemetry.update();
//            }
//        }
//        telemetry.addData("Rad", rad);
//        telemetry.update();
        apache.setPose(new Pose2d(0, 0, new Rotation2d(0)));
    }

    @Override
    public void main() {
        sleep(1000);
        timer.reset();
        while (timer.seconds() < 300 && free == false) {
            telemetry.addData("PosX", apache.getLastReceivedCameraUpdate().pose.getX() * 39.3701);
            telemetry.addData("PosY", apache.getLastReceivedCameraUpdate().pose.getY() * 39.3701 - 72);
            telemetry.addData("Deg", apache.getLastReceivedCameraUpdate().pose.getRotation().getDegrees());
            telemetry.addData("Rad", apache.getLastReceivedCameraUpdate().pose.getRotation().getRadians());
            telemetry.update();
            if (isStopRequested()) {
                apache.stop();
                free = true;
            }
        }
    }
}
