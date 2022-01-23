package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class RedLeft extends FishloAutonomousProgram {

    String position;
    ElapsedTime timer;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        timer = new ElapsedTime();
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

        drive.driveleft(10, 0.8, false, 0);

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
        //Moving towards shipping hub
        drive.drive(20 , 0.6, false, 0);
        sleep(200);
        telemetry.addLine("5. Drop off block");
        telemetry.update();
        //drop block onto shipping hub
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(1000);
        intake.intake(Intake.IntakeState.OFF);
        //turning the robot parallel with wall
        drive.drive(-26 , 0.5, false, 0);
        intake.armToLevel(3);
        intake.stop();
        intake.resetEncoder();
        drive.driveleft(17,0.6,false, 0);
        sleep(200);
        drive.drive(-47 , 0.5, false, 0);
        drive.driveleft(1, -0.4,true, 1.5);
        intake.duck.setPower(-0.6);
        sleep(4000);
        intake.duck.setPower(0);
        drive.driveRight(5,0.5,true,1);
        sleep(200);
        telemetry.addLine("Forward");
        telemetry.update();
        drive.drive(20, 0.7, false, 0);
        telemetry.addLine("Done");
        telemetry.update();
        sleep(3000);
    }
}

