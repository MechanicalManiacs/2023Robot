package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

import com.spartronics4915.lib.T265Camera;

@Autonomous
public class T265Test extends FishloAutonomousProgram {

    T265Camera apache;
    T265Camera.CameraUpdate up;
    double rad;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.setAutoClear(true);
        apache = new T265Camera(new Transform2d(new Translation2d(-0.1143,0), new Rotation2d(0)), 0, hardwareMap.appContext);
        apache.start();
        telemetry.addLine("Ready for start");
        telemetry.update();
        rad = apache.getLastReceivedCameraUpdate().pose.getRotation().getRadians();
        while (!isStarted()) {
            if (rad > 0) {
                rad -= 0.01;
                telemetry.addData("Rad", rad);
                telemetry.update();
            } else if (rad < 0) {
                rad += 0.01;
                telemetry.addData("Rad", rad);
                telemetry.update();
            }
        }
        telemetry.addData("Rad", rad);
        telemetry.update();
        apache.setPose(new Pose2d(0, 0, new Rotation2d(rad)));
    }

    @Override
    public void main() {
        apache.setPose(new Pose2d(0,0,new Rotation2d(rad)));
        telemetry.addLine("Camera heading 0");
        telemetry.update();
        sleep(1000);
        while (!isStopRequested()) {
            telemetry.addData("PosX", apache.getLastReceivedCameraUpdate().pose.getX() * 39.3701 + 5);
            telemetry.addData("PosY", apache.getLastReceivedCameraUpdate().pose.getY() * 39.3701 * 1.0625 - 72);
            telemetry.addData("Deg", apache.getLastReceivedCameraUpdate().pose.getRotation().getDegrees());
            telemetry.addData("Rad", apache.getLastReceivedCameraUpdate().pose.getRotation().getRadians());
            telemetry.update();
        }
    }
}
