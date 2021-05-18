
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "RedLeftMax", group = "Taus")
public class RedLeftMax extends AutonomousMethods {

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
        controlArmServo(0);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 16);
                shoot(0.0, (2215.0*28)/60.0);
                forward(.5, 0, 14);
                strafeRight(.5, 1, 0);
                toAngle(180, 1);
                dropWobbleGoal();
                forward(.5, 1, 20);
                toAngle(260, 1);
                pickUpWobbleGoal(4);
                toAngle(0, 1);
                forward(.5, 1, 12);
                toAngle(180, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 0);
                toAngle(0, 1);
                forward(.5, 0, 12);

                break;
            case 1:
                //code
                forward(.5, 2, 16);
                shoot(0.0, (2214.2*28)/60.0);
                forward(.5, 1, 14);
                toAngle(180, 1);
                dropWobbleGoal();
                forward(.5, 1, 2);
                toAngle(330, 1);
                setIntakePower(0);
                backward(.5, 1, 20);
                toAngle(255, 1);
                pickUpWobbleGoal(6);
                toAngle(0, 1);
                forward(.5, 1, 8);
                shoot(-21.0, (2217.8*28)/60.0);
                forward(.5, 0, 16);
                toAngle(90, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 4);
                toAngle(0, 1);

                break;
            case 4:
                //code
                forward(.5, 5, 0);
                toAngle(180, 1);
                strafeLeft(.5, 1, 0);
                dropWobbleGoal();
                strafeRight(.5, 0, 4);
                toAngle(360, 1);
                setIntakePower(1);
                backward(.5, 2, 21);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 6);
                shoot(-20.0, (2268.2*28)/60.0);
                backward(.5, 0, 19);
                toAngle(-110, 1);
                pickUpWobbleGoal(8);
                toAngle(0, 1);
                forward(.5, 1, 11);
                shoot(-20.0, (2217.3*28)/60.0);
                toAngle(0, 1);
                forward(.5, 2, 4);
                toAngle(180, 1);
                dropWobbleGoal();
                forward(.5, 1, 16);
                toAngle(0, 1);

                break;
        }

    }
}
