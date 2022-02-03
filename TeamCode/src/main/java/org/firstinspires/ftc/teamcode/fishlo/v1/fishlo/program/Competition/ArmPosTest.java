package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class ArmPosTest extends FishloAutonomousProgram {
    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        intake.resetEncoder();
        telemetry.setAutoClear(true);
        telemetry.addLine("start the damn prorgam");
        telemetry.update();
    }

    @Override
    public void main() {
        telemetry.clear();
        telemetry.addLine("level 0");
        telemetry.update();
        intake.armToLevel(0, false, 0);
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(2000);
        intake.intake(Intake.IntakeState.OFF);
        sleep(5000);
        intake.armToLevel(3, false, 0);
        intake.resetEncoder();
        intake.armToLevel(1, false, 0);
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(2000);
        intake.intake(Intake.IntakeState.OFF);
        sleep(5000);
        intake.armToLevel(3, false, 0);
        intake.resetEncoder();
        intake.armToLevel(2,false,0);
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(2000);
//        telemetry.addLine("level 1");
//        intake.armToLevel(1);
//        sleep(200);
//        telemetry.addLine("level 2");
//        intake.armToLevel(2);
    }
}
