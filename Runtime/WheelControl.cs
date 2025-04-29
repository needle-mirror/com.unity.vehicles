using Unity.Entities;

namespace Unity.Vehicles
{
    /// <summary>
    /// Control settings for the wheel.
    /// </summary>
    public struct WheelControl : IComponentData
    {
        /// <summary>
        /// Maximum wheel steer angle, in degrees. 
        /// Can be negative to flip the steer direction.
        /// </summary>
        public float MaxSteerAngle;

        /// <summary>
        /// This value represents the additional steering angle that will be applied
        /// to the wheel on the inside of the turning circle.
        /// There is no effect if the wheel does not share an axle with an another wheel.
        /// Set to 0 to disable.
        /// </summary>
        public float AckermanCoefficient;

        /// <summary>
        /// The amount of motor torque this wheel can receive as a percentage
        /// of the maximum motor torque available for this wheel.
        /// </summary>
        public float MotorTorqueCoefficient;

        /// <summary>
        /// The amount of brake torque this wheel can receive as a percentage
        /// of the MaxBrakeTorque.
        /// </summary>
        public float BrakeTorqueCoefficient;

        /// <summary>
        /// Determines the strength of the handbrake for this wheel
        /// as a percentage of the MaxBrakeTorque.
        /// </summary>  
        public float HandbrakeTorqueCoefficient;

        /// <summary>
        /// The amount of spin resistance torque this wheel has when no motor torque is applied.
        /// </summary>
        public float SpinResistanceTorque;
    }
}