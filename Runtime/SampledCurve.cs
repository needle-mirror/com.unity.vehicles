using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

namespace Unity.Vehicles
{
    /// <summary>
    /// A struct that allows converting an AnimationCurve to a SampledCurve blob asset.
    /// </summary>
    [Serializable]
    public struct SampledAnimationCurve
    {
        /// <summary>
        /// AnimationCurve that will be sampled.
        /// </summary>
        public UnityEngine.AnimationCurve AnimationCurve;

        /// <summary>
        /// Number of samples along the AnimationCurve.
        /// </summary>
        public int SamplesCount;
        
        /// <summary>
        /// Creates the blob asset for this curve
        /// </summary>
        /// <param name="baker">The baker</param>
        /// <returns>The blob asset reference to the sampled curve blob</returns>
        public BlobAssetReference<SampledCurve> CreateBlob(IBaker baker)
        {
            // Minimum samples is 2: one at start and one at the end
            SamplesCount = math.max(2, SamplesCount);

            BlobBuilder builder = new BlobBuilder(Allocator.Temp);
            ref SampledCurve sampledCurve = ref builder.ConstructRoot<SampledCurve>();

            // Create curve
            sampledCurve.XRangeMinMax = default;
            if (AnimationCurve.length == 1)
            {
                sampledCurve.XRangeMinMax = new float2
                {
                    x = AnimationCurve.keys[0].time,
                    y = AnimationCurve.keys[0].time,
                };
            }
            else if (AnimationCurve.length > 1)
            {
                sampledCurve.XRangeMinMax = new float2
                {
                    x = AnimationCurve.keys[0].time,
                    y = AnimationCurve.keys[AnimationCurve.length - 1].time,
                };
            }

            BlobBuilderArray<float> valuesBuilder = builder.Allocate(ref sampledCurve.Values, SamplesCount);
            float xIncrements = math.distance(sampledCurve.XRangeMinMax.x, sampledCurve.XRangeMinMax.y) /
                                (SamplesCount - 1);
            for (int i = 0; i < SamplesCount; i++)
            {
                float result = AnimationCurve.Evaluate(i * xIncrements);
                valuesBuilder[i] = math.select(result, 0f, float.IsNaN(result));
            }

            BlobAssetReference<SampledCurve> blobReference =
                builder.CreateBlobAssetReference<SampledCurve>(Allocator.Persistent);
            baker.AddBlobAsset(ref blobReference, out var hash);
            builder.Dispose();

            return blobReference;
        }
    }

    /// <summary>
    /// Curve blob asset.
    /// </summary>
    public struct SampledCurve
    {
        /// <summary>
        /// Sampled points along the curve.
        /// </summary>
        public BlobArray<float> Values;

        /// <summary>
        /// Range of the values on the X-axis.
        /// </summary>
        public float2 XRangeMinMax;

        /// <summary>
        /// Returns the value of the curve at the given time.
        /// </summary>
        /// <param name="x">Time at which the curve will be sampled.</param>
        /// <returns>Value at the given time.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float EvaluateAt(float x)
        {
            x = math.clamp(x, XRangeMinMax.x, XRangeMinMax.y);
            float normalizedX = math.saturate((x - XRangeMinMax.x) / (XRangeMinMax.y - XRangeMinMax.x));
            float xIndexDecimal = normalizedX * math.max(0f, Values.Length - 1f);
            float ratio = xIndexDecimal % 1f;
            int2 indexes = new int2
            {
                x = (int)math.floor(xIndexDecimal),
                y = (int)math.ceil(xIndexDecimal),
            };
            indexes = math.clamp(indexes, 0, Values.Length - 1);
            return math.lerp(Values[indexes.x], Values[indexes.y], ratio);
        }
    }
}