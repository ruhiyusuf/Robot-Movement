// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);

  m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", -(m_stick->GetRawAxis(1)));
  frc::SmartDashboard::PutNumber("right x: ", m_stick->GetRawAxis(4));
  
  frc::SmartDashboard::PutNumber("currPSI", PSI);

}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  pressed_button_pressure = false;
  valve_start = false;
  
  valve.Set(false);
  frc::SmartDashboard::PutBoolean("valve", false);

  v_shutoffTimerFlag = false;
  v_shutoffTimer->Reset();

  c_shutoffTimerFlag = false;
  c_Timer->Reset();

  compressor_start = false;
  compressed_button_pressed = false;

  v_shutoffDelay = -1; // -1 is default behavior, non zero value is delayed shutoff in seconds

  count = 0;
}
void Robot::TeleopPeriodic() {
  left_y = m_stick->GetRawAxis(1);
  right_x = m_stick->GetRawAxis(4);

  m_robotDrive->ArcadeDrive(-left_y, right_x);

  frc::SmartDashboard::PutNumber("speedLeft", m_leftFollowMotor->Get());
  frc::SmartDashboard::PutNumber("speedRight", m_rightFollowMotor->Get());

  frc::SmartDashboard::PutNumber("C_TIMER", c_Timer->Get());

  // c_shutoffDelay = frc::SmartDashboard::GetNumber("compressorDelay", 21.0);
  frc::SmartDashboard::PutNumber("compressorDelay", c_shutoffDelay);

  analog_input->GetVoltage();
  frc::SmartDashboard::PutNumber("analogInput", analog_input->GetVoltage());

  var_input = frc::SmartDashboard::GetNumber("varInput", 1);
  frc::SmartDashboard::PutNumber("varInput", var_input);

  maxPSI = frc::SmartDashboard::GetNumber("maxPSI", 82);
  frc::SmartDashboard::PutNumber("maxPSI", maxPSI);

  v_shutoffDelay = frc::SmartDashboard::GetNumber("valveDelay", 0.25);
  frc::SmartDashboard::PutNumber("valveDelay", v_shutoffDelay);

  PSI = (analog_input->GetVoltage()) * 100 + 10; // transfer function

  c_shutoffDelay = (0.3836 * maxPSI) -9.4122; // transfer function

  // to compress
  if (m_stick->GetRawButtonPressed(1)) {
    valve.Set(false);
    compressor_start = true;
    compressed_button_pressed = true;
    frc::SmartDashboard::PutBoolean("triggerpress", true);
    frc::SmartDashboard::PutBoolean("valve", false);
  }

  if (m_stick->GetRawButtonPressed(3)) {
    valve.Set(false);
    frc::SmartDashboard::PutBoolean("valve", false);
  }

  if (m_stick->GetRawButtonPressed(4)) {
    compressor->Set(0);
    frc::SmartDashboard::PutBoolean("valve", false);
  }

  if (m_stick->GetRawButtonPressed(2)) {
    valve.Set(true);
    valve_start = true;
    frc::SmartDashboard::PutBoolean("valve", true);
  }

  if (valve_start) {
    v_shutoffTimer->Reset();
    v_shutoffTimer->Start();

    valve_start = false;
  } else if (v_shutoffTimer->Get() >= v_shutoffDelay) {
    v_shutoffTimer->Stop();
    valve.Set(false);
    frc::SmartDashboard::PutBoolean("valve", false);
    std::cout << "VALVE: stopped timer" << std::endl;
  }

  if (compressor_start && compressed_button_pressed) {
    c_Timer->Reset();
    c_Timer->Start();

    compressor_start = false;
  } else if (compressed_button_pressed) {
   
    if (c_Timer->Get() < c_shutoffDelay) {
      compressor->Set(1);
    } else if (c_Timer->Get() >= c_shutoffDelay) {
      compressor->Set(0);
      pressed_button_pressure = false; 
      frc::SmartDashboard::PutBoolean("triggerpress", false);
      compressed_button_pressed = false;

      c_Timer->Stop();
    } 
    
  }
  
  
  // if button pressed,
  // include if statement
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif