package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.intel.realsense.librealsense.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.TSEDetectionPipeline;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class VisionTest extends FishloAutonomousProgram {

    TSEDetectionPipeline.BarcodePosition placement;
    TSEDetectionPipeline mainPipeline;
    OpenCvCamera cam;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.addLine("start");
        telemetry.setAutoClear(true);
        while (!isStarted()) {
            int cameraViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMoniterViewId", "id", hardwareMap.appContext.getPackageName());
            cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraViewID);

            cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    mainPipeline = new TSEDetectionPipeline();
                    cam.setPipeline(mainPipeline);
                    cam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("camera error", "");
                    System.exit(0);
                }
            });
            /*placement = vision.getPlacement();
            telemetry.addData("Position", placement);
            telemetry.update();*/
            placement = mainPipeline.getBarcodePosition();
            telemetry.addData("Position", placement);
            telemetry.update();
        }
    }

    @Override
    public void main() {
        super.main();
    }
}
