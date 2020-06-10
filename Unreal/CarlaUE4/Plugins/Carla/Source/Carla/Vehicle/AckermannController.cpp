// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "AckermannController.h"

#include "Vehicle/CarlaWheeledVehicle.h"

#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
#include "WheeledVehicleMovementComponent.h"

// =============================================================================
// -- Constructor and destructor -----------------------------------------------
// =============================================================================

AAckermannController::AAckermannController(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PostPhysics;
}

AAckermannController::~AAckermannController() {}

// =============================================================================
// -- AController --------------------------------------------------------------
// =============================================================================

void AAckermannController::OnPossess(APawn *aPawn)
{
  Super::OnPossess(aPawn);

  if (IsPossessingAVehicle())
  {
    UE_LOG(LogCarla, Error, TEXT("Controller already possessing a vehicle!"));
    return;
  }
  Vehicle = Cast<ACarlaWheeledVehicle>(aPawn);
  check(Vehicle != nullptr);
}

void AAckermannController::OnUnPossess()
{
  Super::OnUnPossess();

  Vehicle = nullptr;
}

void AAckermannController::Tick(const float DeltaTime)
{
  Super::Tick(DeltaTime);

  if (!IsPossessingAVehicle())
  {
    return;
  }

  UE_LOG(
      LogCarlaServer,
      Log,
      TEXT("Tick Ackermann controller: Current status(v=%f, a=%f, max_steer=%f)"),
      GetCurrentSpeed(),
      GetCurrentAcceleration(DeltaTime),
      Vehicle->GetMaximumSteerAngle());

  //RunControlLoop();

  FVehicleControl OutControl;
  OutControl.Steer = OutSteer;
  OutControl.Throttle = OutThrottle;
  OutControl.Brake = OutBrake;
  OutControl.bHandBrake = OutHandBrake;
  OutControl.bReverse = OutReverse;

  PreviousVelocity = GetCurrentSpeed();

  Vehicle->ApplyVehicleControl(OutControl, EVehicleInputPriority::Relaxation);
  Vehicle->FlushVehicleControl();
}

void AAckermannController::RunControlLoop()
{
  OutSteer = TargetSteer;
  OutThrottle = 1.0;
  OutBrake = 0.0;
  OutHandBrake = false;
  OutReverse = false;

  UE_LOG(
      LogCarlaServer,
      Log,
      TEXT("Current command applied (steer=%f, throttle=%f, brake=%f)"),
      OutSteer,
      OutThrottle,
      OutBrake);
}

void AAckermannController::RunSteerControlLoop()
{
  OutSteer = TargetSteer / GetMaximumSteerAngle();
}

void AAckermannController::RunSpeedControlLoop()
{
  //SpeedController;
}

void AAckermannController::RunAccelControlLoop()
{
  // Setpoint of the acceleration controller is the output of the speed controller.
  //AccelerationController.SetPoint(SpeedControlAccelTarget);
  //AccelControlPedalDelta = AccelerationController.Run(GetCurrentAcceleration());
  
  float MaximumPedal = GetMaximumPedal();
  AccelControlPedalTarget += AccelControlPedalDelta;
  AccelControlPedalTarget = std::max(-MaximumPedal, std::min(AccelControlPedalTarget, MaximumPedal));
}

float AAckermannController::GetCurrentSpeed()
{
  return Vehicle->GetVehicleForwardSpeed() * 10e-3;
}

float AAckermannController::GetCurrentAcceleration(float DeltaTime)
{
  float Acceleration = (GetCurrentSpeed() - PreviousVelocity) / DeltaTime;
  return Acceleration;
}

float AAckermannController::GetMaximumSteerAngle() const
{
  return (Vehicle->GetMaximumSteerAngle() * 3.14f) / 180.0f;
}

float AAckermannController::GetMaximumSpeed() const
{
  // 180 km/h is the default maximum speed of a car.
  return 180.0f / 3.6f;
}

float AAckermannController::GetMaximumDeceleration() const
{
  return -8.0f;
}

float AAckermannController::GetMaximumAcceleration() const
{
  return 3.0f;
}

float AAckermannController::GetMaximumPedal() const
{
  return 0.0f;
}

void AAckermannController::SetTargetSteer(float Steer)
{
  float MaximumSteerAngle = GetMaximumSteerAngle();
  if (std::abs(Steer) < MaximumSteerAngle)
  {
    UE_LOG(LogCarlaServer, Log, TEXT("Max steering angle reached, clipping value"));
  }

  TargetSteer = std::max(-MaximumSteerAngle, std::min(Steer, MaximumSteerAngle));
}

void AAckermannController::SetTargetSteerSpeed(float SteerSpeed)
{
  TargetSteerSpeed = SteerSpeed;
}

void AAckermannController::SetTargetSpeed(float Speed)
{
  float MaximumSpeed = GetMaximumSpeed();
  if (std::abs(Speed) < MaximumSpeed)
  {
    UE_LOG(LogCarlaServer, Log, TEXT("Max speed reached, clipping value"));
  }

  TargetSpeed = std::max(-MaximumSpeed, std::min(Speed, MaximumSpeed));
}

void AAckermannController::SetTargetAcceleration(float Acceleration)
{
  float MaximumDeceleration = GetMaximumDeceleration();
  float MaximumAcceleration = GetMaximumAcceleration();
  if (std::abs(TargetSpeed) < 0.00001f)
  {
    UE_LOG(LogCarlaServer, Log, TEXT("Target velocity set to zero, using max decel value"));
    TargetAcceleration = MaximumDeceleration;
  }

  if (Acceleration < MaximumDeceleration || Acceleration > MaximumAcceleration)
  {
    UE_LOG(LogCarlaServer, Log, TEXT("Max acceleration reached, clipping value"));
  }

  TargetAcceleration = std::max(MaximumDeceleration, std::min(Acceleration, MaximumDeceleration));
}

void AAckermannController::SetTargetJerk(float Jerk)
{
  TargetJerk = Jerk;
}

void AAckermannController::ApplyVehicleControl(const FVehicleControlAckermann &InControl)
{
  UE_LOG(LogCarla, Error, TEXT("Modifying target point"));

  SetTargetSteer(InControl.Steer);
  SetTargetSteerSpeed(InControl.SteerSpeed);
  SetTargetSpeed(InControl.Speed);
  SetTargetAcceleration(InControl.Acceleration);
  SetTargetJerk(InControl.Jerk);
}

