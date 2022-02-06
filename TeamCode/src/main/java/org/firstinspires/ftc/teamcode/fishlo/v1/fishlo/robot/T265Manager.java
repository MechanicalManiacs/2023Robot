package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

public class T265Manager {
    private static Transform2d cameraRobotOffset = new Transform2d();
    private static double odometryCovariance = 0;

    private static T265Camera apache = null;

    public static T265Camera get(HardwareMap hardwareMap) {
        if (apache == null) {
            apache = new T265Camera(cameraRobotOffset, odometryCovariance, hardwareMap.appContext);
        }
        return apache;
    }
}
