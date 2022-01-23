package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
public class RedRight extends FishloAutonomousProgram {
    String position;
    ElapsedTime timer;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        intake.resetEncoder();
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

        drive.driveRight(10, 0.8, false, 0);

        sleep(200);

        telemetry.addLine("2. Squaring up with wall");
        telemetry.update();
        telemetry.addLine("3. Move arm to position");
        telemetry.update();

        switch(position) {
            case "Left":
                intake.resetEncoder();
                intake.armToLevel(0);
                break;
            case "Center":
                intake.resetEncoder();
                intake.armToLevel(1);
                break;
            case "Right":
                intake.resetEncoder();
                intake.armToLevel(2);
                break;
        }

        telemetry.addLine("4. Move up to the shipping hub");
        telemetry.update();
        //Moving towards shipping hub
        drive.drive(17, 0.6, false, 0);
        sleep(200);
        telemetry.addLine("5. Drop off block");
        telemetry.update();
        //drop block onto shipping hub
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(1000);
        intake.intake(Intake.IntakeState.OFF);
        intake.stop();
        //turning the robot parallel with wall
        drive.drive(-22 , 0.5, false, 0);
        intake.resetEncoder();
        drive.driveRight(-37,0.6, false, 0);
        intake.intake(Intake.IntakeState.ON);
        drive.drive(70, 0.7, false, 0);
        drive.drive(-10,0.5, false, 0);
        drive.strafe(5, 0.5, true, 1);
        drive.drive(-72, 0.7, false, 0);
        intake.intake(Intake.IntakeState.OFF);
//        sleep(200);
//        drive.drive(-47 , 0.5);
//        drive.driveleft(1, -0.4, true, 1.5);
//        telemetry.addLine("starting to spin the duck");
//        telemetry.update();
//        intake.duck.setPower(-0.6);
//        sleep(5000);
//        intake.duck.setPower(0);
//        telemetry.addLine("finished spining the duck");
//        telemetry.update();
//        drive.drive(15, 0.7);
//        drive.drive(22, 0.6);
    }
}
