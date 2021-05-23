//Run code for the blue left position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "BlueLeft2", group = "Taus")
public class BlueLeft2 extends AutonomousMethods {

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
                forward(.5, 2, 11);
                shoot(0, (2150*28)/60.0);
                toAngle(60, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(0, 1);
                backward(.5, 1, 0);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(0, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 18);//12
                toAngle(-160, 1);
                dropWobbleGoal();
                toAngle(-180, 1);
                strafeRight(.5, 0, 2);
                forward(.5, 2, 12);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(.5, 0, 12);
                forward(.5, 0, 9);
                toAngle(0, 1);
                forward(.5, 0, 23);
                shoot(0, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 11);
                break;
            case 4:
                //code
                forward(.5, 1, 16);
                strafeLeft(.5, 0, 2);
                shoot(-2, (2250*28)/60.0);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(.5, 0, 12);
                setIntakePower(-.3);
                forward(.5, 0, 2);
                setIntakePower(1);
                backward(.5, 0, 4);
                setIntakePower(-.3);
                forward(.5, 0, 2);
                setIntakePower(1);
                backward(.5, 0, 4);
                setIntakePower(-1);
                setIntakePower(1);
                forward(.5, 0, 12);//6
                shoot(-4, (2250*28)/60.0);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(.5, 0, 17);
                forward(.5, 0, 16);//17
                toAngle(0, 1);
                forward(.5, 0, 23);
                shoot(-2, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.7, 1, 18);
                toAngle(60, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 2);
                toAngle(0, 1);
                backward(.5, 1, 4);
                break;
        }

    }
}
