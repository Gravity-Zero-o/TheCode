## README 
#### We use an open-source hardware platform named  Arduino.
And the following are some explanations.

To start with, we use a acceleration sensor whose name is MPU 6050 to gain the change of acceleration. As the object dropping down, the sensor will know that the z-axis acceleration do not be zero but is 9.8. To reach this function, we use a judge function to judge whether the z-axis acceleration become the number between 9 and 10.

The second step, we use the step-motor to open the lid before one second the umbrella being released. The motor open needs just 0.5s.

And then, the spring released and push the bag of umbrella out of the cave, with the force of wind, the umbrella open.

Also, we do have a method to protect our machine. If the Ultrasonic sensor find it too close to the floor, we set it as 4 meters, the balloon will be released to defend the crash.




