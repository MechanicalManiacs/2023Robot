package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class VisionTest extends FishloAutonomousProgram {

    String placement;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.addLine("start");
        telemetry.setAutoClear(true);
        while (!isStarted()) {
            placement = vision.getPlacement();
        }
    }

    @Override
    public void main() {
        super.main();
    }
}
