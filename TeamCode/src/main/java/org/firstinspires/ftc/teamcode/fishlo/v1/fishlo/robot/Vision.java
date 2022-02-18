package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvWebcam;

public class Vision extends SubSystem {

    private OpenCvCamera webcam;
    private VisionPipeline pipeline2;

    private TSEDetectionPipeline.BarcodePosition barcodePosition = TSEDetectionPipeline.BarcodePosition.NULL;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.32; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.82; //i.e 60% of the way across the frame from the left

    // Pink Range                                      Y      Cr     Cb

    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public Vision(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {

    }

    public void initVision() {
        // OpenCV webcam
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);

        //OpenCV Pipeline

        pipeline2 = new VisionPipeline(0, 0, 0, 0);

        pipeline2.configureScalarLower(VisionPipeline.scalarLowerHSV.val[0], VisionPipeline.scalarLowerHSV.val[1], VisionPipeline.scalarLowerHSV.val[2]);
        pipeline2.configureScalarUpper(VisionPipeline.scalarUpperHSV.val[0], VisionPipeline.scalarUpperHSV.val[0], VisionPipeline.scalarUpperHSV.val[0]);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline2);
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.exit(0);
            }
        });


    }

    public TSEDetectionPipeline.BarcodePosition getPlacement() {
        if (pipeline2.getRectMidpointX() > TSEDetectionPipeline.rightThreshold) {
            barcodePosition = TSEDetectionPipeline.BarcodePosition.RIGHT;
        }
        else if (pipeline2.getRectMidpointX() > TSEDetectionPipeline.leftThreshold) {
            barcodePosition = TSEDetectionPipeline.BarcodePosition.LEFT;
        }
        else {
            barcodePosition = TSEDetectionPipeline.BarcodePosition.CENTER;
        }
        return barcodePosition;
    }

    public double getVisionX() {
        return pipeline2.getRectMidpointX();
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        webcam.closeCameraDevice();
    }
}
