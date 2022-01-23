package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class Strafetest extends FishloAutonomousProgram {
    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.addLine("start");
        telemetry.update();
    }

    @Override
    public void main() {
        telemetry.addLine("Right");
        telemetry.update();
        drive.strafe(5, 1, false, 0);
        sleep(3000);
        telemetry.addLine("Left");
        telemetry.update();
        drive.strafe(-5, 1, false, 0);
    }
}
