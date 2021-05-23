//Run code for the red left position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "RedLeft2", group = "Taus")
public class RedLeft2 extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.25);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 10);//16
                shoot(0, (2150*28)/60.0);
                forward(.5, 0, 18);//12
                strafeRight(.5, 1, 0);
                toAngle(180, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 0);
                toAngle(0, 1);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(0, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 21);//15
                toAngle(-150, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(-180, 1);
                forward(.5, 2, 15);
                toAngle(270, 1);
                setIntakePower(1);
                backward(.5, 0, 12);
                forward(.5, 0, 12);
                toAngle(0, 1);
                forward(.5, 0, 20);
                shoot(2, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
            case 4:
                //code
                forward(.5, 1, 14);
                strafeLeft(.5, 0, 3);
                shoot(0, (2250*28)/60.0);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(.5, 0, 8);//12
                setIntakePower(-.3);
                forward(.5, 0, 2);
                setIntakePower(1);
                backward(.5, 0, 4);
                setIntakePower(-.3);
                forward(.5, 0, 2);
                setIntakePower(1);
                backward(.5, 0, 4);
                forward(.5, 0, 12);//6
                shoot(-2, (2250*28)/60.0);
                setIntakePower(1);
                toAngle(-90, 1);
                backward(.5, 0, 16);
                setIntakePower(-1);
                setIntakePower(1);
                forward(.5, 0, 8);//17
                toAngle(0, 1);
                forward(.5, 0, 23);
                shoot(0, (2150*28)/60.0);
                forward(.5, 2, 8);
                strafeRight(.5, 1, 0);
                toAngle(155, 1);
                dropWobbleGoal();
//                strafeRight(.5, 2, 0);
//                backward(.5, 1, 0);
//                toAngle(0, 1);
                break;
        }

    }
}
