# 2026RobotCode

## To do:
 <!-- 
 ! general: 
 !  - reformat maplesim default files
 !  - tune pid without motionmagic first
 !  - tune pid through phoenixtuner https://discord.com/channels/176186766946992128/368993897495527424/1438631090578259968
 ! driving: 
 !  - sysid
 !  - add back the wheel characterization functionality
 !  - review colin's aiming pid and https://github.com/FRC-4481-Team-Rembrandts/4481-robot-2024-public/blob/main/src/main/java/frc/team4481/robot/autoaim/TurnToPose.java 
 !  - global vs. local positioning
 !  - rotating around a swerve module
 ! vision: 
 !  - apriltag 1 foot away from robot for camera on robot position
 !  - object detection
 !  - onboard video recording
 -->

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
 - Controllers: 
     - To remove controllers from Driver Station, double click on them
        - This is helpful when pairing new controllers and they reach the limit of how many controllers DS shows
     - Sometimes, a more niche controller (e.g. Gamesir) might show up as multiple controllers in DS
 - MrCal: 
     - https://docs.google.com/document/d/1SDDetHYSshZeu1tcnOC5SHEPUEZCRtlPO9kb6NCubQs/edit?usp=sharing