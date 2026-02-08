package frc.robot.subsystems.fuelIO.shooter;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {
    

    private final DCMotor flywheelMotor = DCMotor.getKrakenX60(1);
    private final FlywheelSim flywheelSim =
            new FlywheelSim(LinearSystemId.createFlywheelSystem(flywheelMotor, 0.005, 2), flywheelMotor, 0.0005);

    

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.update(0.02);
    
        
        inputs.shooterVelocity = flywheelSim.getAngularVelocity().magnitude();
        inputs.shooterCurrent = flywheelSim.getCurrentDrawAmps();
    }

    @Override
    public  void setVoltage(Voltage volts) {
        flywheelSim.setInputVoltage(volts.magnitude());
    }


    @Override
    public  void setVelocity(AngularVelocity velocity) {
        flywheelSim.setAngularVelocity(velocity.magnitude());
    }

    // @Override
    // public  void fire() {
        
    // }
}