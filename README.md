# 2026RobotCode

## To do:
 <!-- 
 ! general: 
 !  - reformat maplesim default files
 !  - tune pid without motionmagic first
 !  - tune pid through phoenixtuner https://discord.com/channels/176186766946992128/3689938974955274241438631090578259968
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
     - After being flashed, IP address should be 10.32.5.1
     - For configuration, don't use their quick start guide, just power with 12V 2A through a VRM and go to the IP address
     - https://www.youtube.com/watch?v=eQYytkdBkt4
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
 - CAN IDs: 
     - Drivebase motors are 11, 12, 21, 22, 31, 32, 41, 42 (units digit 1 is drive, 2 is turn)
     - Drivebase CANCoders are 1, 2, 3, 4
     - Pigeon is 5
     - Subsystems are 50, 51, ...
 - Phoenix Tuner X: 
     - Must be running robot code to show any devices
 - RevLib: 
    - Make sure version is fully updated before trying to flash firmware
    - To set up a Sparkmax, turn off all power, hold down reset button, and plug it into computer through usb-c
    - Should show up as Unknown Device in Rev Hardware Client and then you can format it from there