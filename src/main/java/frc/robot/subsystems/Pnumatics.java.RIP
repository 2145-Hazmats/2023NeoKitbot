// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pnumatics extends SubsystemBase {
  /** Creates a new Pnumatics. */
  final Compressor m_comp = new Compressor(PneumaticsModuleType.CTREPCM);

  final DoubleSolenoid m_doubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 1); // 1 and 2

  public Pnumatics() {
    m_comp.disable();
  }

  public void ShootPiston() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void SuckPiston() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void Compress() {
    m_comp.enableDigital();
  }

  // public void StopCompress() {
  //  m_comp.disable();
  // }

  public Command ShootPistonCommand() {
    return this.startEnd(
        () -> {
          ShootPiston();
        },
        () -> {
          ShootPiston();
        });
  }

  public Command SuckPistonCommand() {
    return this.startEnd(
        () -> {
          SuckPiston();
        },
        () -> {
          SuckPiston();
        });
  }

  public Command CompressCommand() {
    return this.startEnd(
        () -> {
          Compress();
        },
        () -> {
          Compress();
        });
  }

  // public Command StopCompressCommand() {
  //  //return this.startEnd(() -> {StopCompress(); },() -> {StopCompress();});
  //  return this.run(
  //    () -> { StopCompress(); }
  //  );
  // }

  @Override
  public void periodic() {}
}
