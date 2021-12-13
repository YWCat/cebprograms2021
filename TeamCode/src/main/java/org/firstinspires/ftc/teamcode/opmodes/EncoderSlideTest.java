package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.FakeGamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@TeleOp
public class EncoderSlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        FakeGamepad gamepad1 = new FakeGamepad();

        telemetry.addLine("Initializing");
        telemetry.update();
        Robot robot = new Robot(this);
        Slide slide = new Slide(robot, telemetry);
        try {
            Thread.sleep(2000);
        }
        catch (InterruptedException e){
            ;
        }
        telemetry.addLine("level (before update): "+ slide.getLevel());
        telemetry.update();
        try {
            Thread.sleep(1000);
        }
        catch (InterruptedException e){
            ;
        }
        robot.registerSubsystem(slide);
        //pause
        try {
            Thread.sleep(1000);
        }
        catch (InterruptedException e){
            ;
        }
        telemetry.addLine("level (init): "+ slide.getLevel());
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            boolean buttonA = gamepad1.a();
            boolean buttonY = gamepad1.y();

            telemetry.addLine("A pressed?"+buttonA +" Y pressed?"+buttonY);
            telemetry.update();

            if (buttonA == true ){
                slide.goDown();
            }
            else if (buttonY == true){
                slide.goUp();
            }

        }
    }
}
