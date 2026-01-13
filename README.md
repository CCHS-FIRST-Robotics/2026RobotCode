# 2026RobotCode

## To do:
 <!--
 ! rename the cancoders and pigeon
 
 ! for arms and elevators, tune pid without motionmagic
 ! reformat io away from being maplesim template
 -->
 - look into pid tunable constants through phoenixtuner
     - https://discord.com/channels/176186766946992128/368993897495527424/1438631090578259968
 - figure out sysid
 - global and local positioning
 - rotating around a swerve module

## Reference:
 - Radios:
     - When first flashed, IP address is 10.0.1.1
     - After being flashed, IP address should be 10.32.5
     - For configuration, don't use their quick start guide, just power with 12V 2A through a VRM and go to the IP address
 - OrangePis:
     - Front dashboard IP address is 10.32.05.16:5800
     - Back dashboard IP address is 10.32.05.17:5800
 - Code: 
     - Template taken from https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced
     - A lot was changed, check 
 - Advantagekit: 
     - Custom assets: https://docs.advantagescope.org/more-features/custom-assets
     - To set these up to be displayed in advantagescope, go to Help > Use Custom Assets Folder and then select the folder called "advantagescope_assets" in this project