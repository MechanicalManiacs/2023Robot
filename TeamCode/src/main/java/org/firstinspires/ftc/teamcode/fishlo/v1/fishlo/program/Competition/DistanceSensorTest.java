package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class DistanceSensorTest extends FishloAutonomousProgram {

    DistanceSensor distanceSensor;
    Intake.BlockIn blockIn = Intake.BlockIn.NOT_IN;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.setAutoClear(true);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        telemetry.addLine("start");
        telemetry.update();
    }

    @Override
    public void main() {
        while (opModeIsActive()) {
            telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.INCH));
            if (distanceSensor.getDistance(DistanceUnit.INCH) <= 1.5) {
                blockIn = Intake.BlockIn.IN;
            }
            else {
                blockIn = Intake.BlockIn.NOT_IN;
            }
            telemetry.addData("Status", blockIn);
            telemetry.update();
        }
    }
}
