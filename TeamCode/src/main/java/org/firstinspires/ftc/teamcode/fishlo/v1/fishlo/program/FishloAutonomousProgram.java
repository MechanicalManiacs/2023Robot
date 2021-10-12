package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Drive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Fishlo;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Vision;
import org.firstinspires.ftc.teamcode.opMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class FishloAutonomousProgram extends AutonomousProgram {

    protected Drive drive;
    protected Vision vision;

    @Override
    protected Robot buildRobot() {
        Fishlo fishlo = new Fishlo(this);

        drive = (Drive) fishlo.getSubSystem("Drive");
        vision = (Vision) fishlo.getSubSystem("Vision");

        return fishlo;
    }

    @Override
    public void main() {}

    @Override
    public void preMain() {}
}