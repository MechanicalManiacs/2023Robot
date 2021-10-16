package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class VisionTest extends FishloAutonomousProgram {

    String data;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        while (!isStarted()) {
            data = vision.getQRCodeData();
        }
    }

    @Override
    public void main() {
        while (!isStopRequested()) {
            telemetry.setAutoClear(true);
            telemetry.addData("Data", data);
            telemetry.addData("Data Validity", vision.validateData(true));
            telemetry.addData("position", vision.getPosition());
        }
    }
}
