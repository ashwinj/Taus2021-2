//Run code for the blue right position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "BlueRight2", group = "Taus")
public class BlueRight2 extends AutonomousMethods {

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
                shoot(-26, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 26);//20
                strafeLeft(.3, 0, 23);
                dropWobbleGoal();
                strafeRight(.5, 1, 4);
                backward(.5, 0, 8);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(-26, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 26);//20
                toAngle(-15,1);
                dropWobbleGoal();
                toAngle(0,1);
                strafeRight(.5, 0, 3);
                backward(.5, 2, 20);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 11);
                forward(.5, 0, 11);
                toAngle(0, 1);
                forward(.5, 0, 20);
                shoot(-26, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
            case 4:
                //code
                forward(.5, 1, 16);
                strafeRight(.5, 0, 3);
                shoot(-26, (2250*28)/60.0);
                toAngle(90, 1);
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
                shoot(-32.0, (2250*28)/60.0);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 16);
                setIntakePower(-1);
                setIntakePower(1);
                forward(.5, 0, 8);//17
                toAngle(0, 1);
                forward(.5, 0, 23);
                shoot(-26, (2150*28)/60.0);
                toAngle(0, 1);
                forward(.5, 2, 10);
                strafeLeft(.5, 0, 23);
                toAngle(45, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 23);
                toAngle(0, 1);
                backward(.5, 0, 12);
                break;
        }

    }
}
