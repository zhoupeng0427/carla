// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "GameFramework/Controller.h"

#include "Vehicle/VehicleControlAckermann.h"

#include "AckermannController.generated.h"

class ACarlaWheeledVehicle;

class PID
{
  public:
    PID() = default;
    PID(float Kp, float Ki, float Kd): Kp(Kp), Ki(Ki), Kd(Kd) {}
    ~PID() = default;

    void SetTargetPoint(float Point)
    {
      SetPoint = Point;
    }

    float Run(float Input, float DeltaTime)
    {
      float Error = SetPoint - Input;

      Proportional = Kp * Error;
      Integral += Ki * Error * DeltaTime;
      Derivative = Kd * (Error - LastError) / DeltaTime;

      // TODO(joel): Restrict to output limits.
      float Out = Proportional + Integral + Derivative;

      return Out;
    }

  private:
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;

    //std::pair<float, float> OutLimits(-1.0f, 1.0f);
    float SetPoint;

    // Internal state.
    float Proportional = 0.0f;
    float Integral = 0.0f;
    float Derivative = 0.0f;

    float LastError = 0.0f;
};


/// Wheeled vehicle ackermann controller
UCLASS()
class CARLA_API AAckermannController final : public AController
{
  GENERATED_BODY()

  // ===========================================================================
  /// @name Constructor and destructor
  // ===========================================================================
  /// @{

public:

  AAckermannController(const FObjectInitializer &ObjectInitializer);

  ~AAckermannController();

  /// @}
  // ===========================================================================
  /// @name Controller overrides
  // ===========================================================================
  /// @{

public:

  void OnPossess(APawn *aPawn) override;

  void OnUnPossess() override;

  void Tick(float DeltaTime) override;

  UFUNCTION(Category = "Ackermann Controller", BlueprintCallable)
  void ApplyVehicleControl(const FVehicleControlAckermann &InControl);

  void RunControlLoop();


  /// @}
  // ===========================================================================
  /// @name Possessed vehicle
  // ===========================================================================
  /// @{

public:

  UFUNCTION(Category = "Ackermann Controller", BlueprintCallable)
  bool IsPossessingAVehicle() const
  {
    return Vehicle != nullptr;
  }

  UFUNCTION(Category = "Ackermann Controller", BlueprintCallable)
  ACarlaWheeledVehicle *GetPossessedVehicle()
  {
    return Vehicle;
  }

  const ACarlaWheeledVehicle *GetPossessedVehicle() const
  {
    return Vehicle;
  }

private:
  float GetCurrentSpeed();
  float GetCurrentAcceleration(float DeltaTime);

  // Restrictions.
  float GetMaximumSteerAngle() const;
  float GetMaximumSpeed() const;
  float GetMaximumDeceleration() const;
  float GetMaximumAcceleration() const;
  float GetMaximumPedal() const;

  void SetTargetSteer(float Steer);
  void SetTargetSteerSpeed(float SteerSpeed);
  void SetTargetSpeed(float Speed);
  void SetTargetAcceleration(float Acceleration);
  void SetTargetJerk(float Jerk);

  void RunSteerControlLoop();
  void RunSpeedControlLoop();
  void RunAccelControlLoop();

private:

  UPROPERTY()
  ACarlaWheeledVehicle *Vehicle = nullptr;

  PID SpeedController = PID(0.0f, 0.0f, 0.0f);
  PID AccelerationController = PID(0.0f, 0.0f, 0.0f);

  // Target values.
  // TODO(joel): This should be a FControlVehicleAckerman?
  float TargetSteer = 0.0f;
  float TargetSteerSpeed = 0.0f;
  float TargetSpeed = 0.0f;
  float TargetAcceleration = 0.0f;
  float TargetJerk = 0.0f;
  //mutable FVector Velocity = {0.0f, 0.0f, 0.0f};

  // Control Output
  // TODO(joel): This should be a FControlVehicle
  float OutSteer = 0.0f;
  float OutThrottle = 0.0f;
  float OutBrake = 0.0f;
  bool OutHandBrake = true;
  bool OutReverse = false;

  // Control values
  int16 SpeedControlActivationCount = 0;
  float SpeedControlAccelDelta = 0.0f;
  float SpeedControlAccelTarget = 0.0f;
  float AccelControlPedalDelta = 0.0f;
  float AccelControlPedalTarget = 0.0f;
  float BrakeUpperBorder = 0.0f;
  float ThrottleLowerBorder = 0.0f;

  // Needed to compute acceleration approximation.
  float PreviousVelocity = 0.0f;
};

