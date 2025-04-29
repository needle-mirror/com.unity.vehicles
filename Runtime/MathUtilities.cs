using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Vehicles
{
    internal static class MathUtilities
    {
        /// <summary>
        /// Generates the points for the wheel ConvexCollider. 
        /// The generated cylinder will always have one edge pointing downwards so that the radius on a flat 
        /// surface matches the provided radius.
        /// </summary>
        /// <param name="points">A NativeArray that holds the generated points.</param>
        /// <param name="numSegments">A number of radial segments around the cylinder.</param>
        /// <param name="radius">External radius of the generated cylinder.</param>
        /// <param name="width">Width of the generated cylinder.</param>
        internal static void GenerateCylinderPoints(ref NativeArray<float3> points, int numSegments, float radius, float width)
        {
            float angleStep = 2 * math.PI / numSegments;
            float halfWidth = width * 0.5f;

            for (int r = 0; r < numSegments; r++)
            {
                float angle = r * angleStep;
                float y = math.cos(angle) * radius;
                float z = math.sin(angle) * radius;
                points[r * 2] = new float3(-halfWidth, y, z);
                points[r * 2 + 1] = new float3(halfWidth, y, z);
            }
        }

        /// <summary>
        /// Projects a vector onto a plane.
        /// </summary>
        /// <param name="vector">Vector to project.</param>
        /// <param name="onPlaneNormal">Normal of the plane.</param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float3 ProjectOnPlane(float3 vector, float3 onPlaneNormal)
        {
            return vector - math.projectsafe(vector, onPlaneNormal);
        }

        /// <summary>
        /// Steps the value towards the target by the step amount.
        /// </summary>
        /// <param name="currentValue">Current value.</param>
        /// <param name="targetValue">Target value.</param>
        /// <param name="step">Largest step allowed.</param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float StepTowards(float currentValue, float targetValue, float step)
        {
            step = math.abs(step);
            float difference = targetValue - currentValue;

            if (difference == 0)
            {
                return currentValue;
            }

            step = math.clamp(step, 0, math.abs(difference));
            return currentValue + math.sign(difference) * step;
        }

        /// <summary>
        /// Returns an interpolant parameter that represents interpolating with a given sharpness
        /// </summary>
        /// <param name="sharpness"> The desired interpolation sharpness </param>
        /// <param name="dt"> The interpolation time delta </param>
        /// <returns> The resulting interpolant </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float GetSharpnessInterpolant(float sharpness, float dt)
        {
            return math.saturate(1f - math.exp(-sharpness * dt));
        }
    }
}