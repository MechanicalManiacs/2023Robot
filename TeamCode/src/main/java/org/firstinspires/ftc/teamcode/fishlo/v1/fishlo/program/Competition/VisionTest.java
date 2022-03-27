package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.acmerobotics.dashboard.config.Config;
import com.intel.realsense.librealsense.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.TSEDetectionPipeline;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Vision;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.VisionPipeline;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public class VisionTest extends FishloAutonomousProgram {

    TSEDetectionPipeline.BarcodePosition placement;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }
        ///hahahhah
    @Override
    public void preMain() {
        telemetry.addLine("start");
        telemetry.update();
        telemetry.setAutoClear(true);
        vision.initVision();
        while (!isStarted()) {
            placement = vision.getPlacement();
            telemetry.addData("Position", placement);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        super.main();
    }
}
