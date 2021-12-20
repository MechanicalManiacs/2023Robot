package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Drive;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Vision.Placement;
@Autonomous
public class MotorEncoderAuto extends FishloAutonomousProgram {

    Placement position = Vision.Placement.CENTER;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        position = vision.getPlacement();

    }
// strafe right is positive power, strafe left is negative power!
//measurements subject to change
    @Override
    public void main() {
        // 1. Drive towards Shipping Hub
        drive.diagonal(10, Drive.Direction.FRONT_RIGHT, 0.5);
        // 2. Move CLoser to shipping Hub
        drive.drive(1,0.2);
        // 3. Lift arm to position and place block
        switch(position) {
            case LEFT:
                intake.armToLevel(0);
                break;
            case CENTER:
                intake.armToLevel(1);
                break;
            case RIGHT:
                intake.armToLevel(2);
                break;
        }
        intake.intake(Intake.IntakeState.REVERSE);
        // 4. Move diagonally backwards to wall
        drive.diagonal(10, Drive.Direction.BACK_LEFT, 0.5);
        // 5. Strafe left to carousel
        drive.strafe(-20,0.5);
        // 6. Spin Carousel Wheel
        intake.spinCarousel(0.5, 10);
        // 7. Move Forward into Storage Unit
        drive.drive(5,0.5);
    }
}

