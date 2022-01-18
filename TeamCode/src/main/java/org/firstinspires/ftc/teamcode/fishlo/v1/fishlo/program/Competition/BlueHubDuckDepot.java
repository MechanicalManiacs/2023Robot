package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class BlueHubDuckDepot extends FishloAutonomousProgram {

    String position;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.setAutoClear(true);
        intake.clawToInitPos();
        telemetry.addLine("Dectecting Position of Barcode");
        while (!isStarted()) {
            if (vision.getPlacement().equals("Left") || vision.getPlacement().equals("Right") || vision.getPlacement().equals("Center")) {
                position = vision.getPlacement();
            }
            telemetry.addData("Position", position);
        }
        telemetry.update();
    }
// strafe right is positive power, strafe left is negative power!
//measurements subject to change
    //Re-push cuz rahul is bad
    @Override
    public void main() {
        vision.stop();
        telemetry.clear();
        telemetry.update();
        telemetry.addLine("1. Strafing");
        telemetry.update();

        drive.driveRight(10, 0.8);

        sleep(200);

        telemetry.addLine("2. Squaring up with wall");
        telemetry.update();
        telemetry.addLine("3. Move arm to position");
        telemetry.update();

        switch(position) {
            case "Left":
                intake.armToLevel(0);
                break;
            case "Center":
                intake.armToLevel(1);
                break;
            case "Right":
                intake.armToLevel(2);
                break;
        }

        sleep(200);

        telemetry.addLine("4. Move up to the shipping hub");
        telemetry.update();
        //drive towards shipping hub
        drive.drive(18, 0.6);
        sleep(200);
        telemetry.addLine("5. Drop off block");
        telemetry.update();
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(1000);
        intake.intake(Intake.IntakeState.OFF);
        intake.armToLevel(3);
        intake.stop();
        intake.resetEncoder();
        sleep(200);

    }
}

