// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.sim.Constants.ArmSim;

/** A robot arm simulation based on a linear system model with Mech2d display. */
public class ArmModel implements AutoCloseable {

  private final ArmSubsystem armSubsystem;
  private double simCurrent = 0.0;
  private CANSparkMaxSim sparkSim;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor armGearbox = DCMotor.getVex775Pro(2);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          armGearbox,
          ArmSim.ARM_REDUCTION,
          SingleJointedArmSim.estimateMOI(ArmSim.ARM_LENGTH_METERS, ArmSim.ARM_MASS_KG),
          ArmSim.ARM_LENGTH_METERS,
          ArmConstants.MIN_ANGLE_RADS,
          ArmConstants.MAX_ANGLE_RADS,
          true,
          ArmSim.START_ANGLE_RADS,
          VecBuilder.fill(ArmSim.ENCODER_DISTANCE_PER_PULSE) // Add noise with a std-dev of 1 tick
          );

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(70, 60);
  private final MechanismRoot2d mechArmPivot = mech2d.getRoot("ArmPivot", 25, 30);
  private final MechanismLigament2d mechArmTower =
      mechArmPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d mechArm =
      mechArmPivot.append(
          new MechanismLigament2d(
              "Arm",
              ArmSim.ARM_LENGTH_INCHES,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  /** Create a new ArmSubsystem. */
  public ArmModel(ArmSubsystem armSubsystemToSimulate) {

    armSubsystem = armSubsystemToSimulate;
    simulationInit();

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", mech2d);
    mechArmTower.setColor(new Color8Bit(Color.kBlue));
  }

  /** Initialize the arm simulation. */
  public void simulationInit() {

    // Setup a simulation of the CANSparkMax and methods to set values
    sparkSim = new CANSparkMaxSim(ArmConstants.MOTOR_PORT);

    // This shouldn't be needed in 2024 since SingleJointedArmSim will allow setting in constructor
    armSim.setState(ArmConstants.ARM_OFFSET_RADS, 0);
  }

  /** Update the simulation model. */
  public void updateSim() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(armSubsystem.getVoltageCommand());

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage and
    // save the current so it can be retrieved later.
    sparkSim.setPosition(armSim.getAngleRads() - ArmConstants.ARM_OFFSET_RADS);
    sparkSim.setVelocity(armSim.getVelocityRadPerSec());
    simCurrent = armSim.getCurrentDrawAmps();
    sparkSim.setCurrent(simCurrent);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    mechArm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    updateShuffleboard();
  }

  /** Return the simulated current. */
  public double getSimCurrent() {
    return simCurrent;
  }

  public void updateShuffleboard() {

    SmartDashboard.putNumber("Arm Sim Angle", Units.radiansToDegrees(armSim.getAngleRads())); // sim
  }

  @Override
  public void close() {
    mech2d.close();
    mechArmPivot.close();
    mechArm.close();
  }
}
