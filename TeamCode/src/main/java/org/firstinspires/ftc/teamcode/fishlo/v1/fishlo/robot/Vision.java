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
    private VisionPipeline pipeline;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.32; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.82; //i.e 60% of the way across the frame from the left

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarUpperHSV = new Scalar(39, 100, 100);
    public static Scalar scalarLowerHSV = new Scalar(54, 95, 90);

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
        // OpenCV webcam
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);

        //OpenCV Pipeline

        pipeline = new VisionPipeline(0, 0, 0, 0);

        pipeline.configureScalarLower(scalarLowerHSV.val[0], scalarLowerHSV.val[1], scalarLowerHSV.val[2]);
        pipeline.configureScalarUpper(scalarUpperHSV.val[0], scalarUpperHSV.val[1], scalarUpperHSV.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


    }

    public String getPlacement() {
        String placement = "";
        if (pipeline.getRectMidpointX() > 360) {
            placement = "Right";

        }
        else if (pipeline.getRectMidpointX() < 230) {
            placement = "Left";
        }
        else {
            placement = "Center";
        }
        return placement;
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        webcam.closeCameraDevice();
    }
}
