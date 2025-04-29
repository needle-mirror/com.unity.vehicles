using Unity.Mathematics;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Vehicles
{
    /// <summary>
    /// Custom property drawer for wheel tire friction curve
    /// </summary>
    [CustomPropertyDrawer(typeof(WheelAuthoring.FrictionCurveData))]
    public class FrictionCurveDrawer : PropertyDrawer
    {
        SerializedProperty m_FrictionCurveProperty;
        SerializedProperty m_FrictionCurveMaximum;
        SerializedProperty m_FrictionCurveMinimum;

        private const float MinCurveXIncrement = 0.001f;

        /// <summary>
        /// On CreatePropertyGUI 
        /// </summary>
        /// <param name="property">The serialized property</param>
        /// <returns>Property visual element</returns>
        public override VisualElement CreatePropertyGUI(SerializedProperty property)
        {
            VisualElement container = new VisualElement();

            // TODO: tooltips

            // Get serialized properties
            m_FrictionCurveMaximum = property.FindPropertyRelative("FrictionCurveMaximum");
            m_FrictionCurveMinimum = property.FindPropertyRelative("FrictionCurveMinimum");

            // Create property field elements
            Vector2Field frictionCurveMaxField = new Vector2Field(m_FrictionCurveMaximum.displayName);
            Vector2Field frictionCurveMinField = new Vector2Field(m_FrictionCurveMinimum.displayName);
            frictionCurveMaxField.SetValueWithoutNotify(m_FrictionCurveMaximum.vector2Value);
            frictionCurveMinField.SetValueWithoutNotify(m_FrictionCurveMinimum.vector2Value);
            frictionCurveMaxField.TrackPropertyValue(m_FrictionCurveMaximum, p =>
            {
                frictionCurveMaxField.SetValueWithoutNotify(m_FrictionCurveMaximum.vector2Value);
            });
            frictionCurveMinField.TrackPropertyValue(m_FrictionCurveMinimum, p =>
            {
                frictionCurveMinField.SetValueWithoutNotify(m_FrictionCurveMinimum.vector2Value);
            });
            SetInputFieldsDelayedInHierarchy(frictionCurveMaxField);
            SetInputFieldsDelayedInHierarchy(frictionCurveMinField);

            // Create curve display elements
            VisualElement frictionCurveContainer = new VisualElement { style = { marginTop = 10, paddingRight = 10, flexDirection = FlexDirection.Row } };
            VisualElement frictionCurveLeft = new VisualElement { style = { width = 15, justifyContent = new StyleEnum<Justify>(Justify.FlexEnd) } };
            VisualElement frictionCurveRight = new VisualElement { style = { flexGrow = 1, flexDirection = FlexDirection.Column } };
            Label curveXLabel = new Label("Slip") { style = { marginLeft = 10 } };
            Label curveYLabel = new Label("Friction Coefficient") { style = { rotate = new Rotate(-90), marginBottom = 5 } };
            VisualElement corner = new VisualElement { style = { width = 15, height = 20 } };
            frictionCurveContainer.Add(frictionCurveLeft);
            frictionCurveContainer.Add(frictionCurveRight);
            CurveField frictionCurveField = new CurveField("Friction Curve") { style = { height = 150, flexGrow = 1 } };
            frictionCurveField.labelElement.style.display = DisplayStyle.None;
            frictionCurveField.SetEnabled(false);

            Vector2 curveMaxVal = m_FrictionCurveMaximum.vector2Value;
            Vector2 curveMinVal = m_FrictionCurveMinimum.vector2Value;
            SanitizeCurveValues(curveMaxVal, curveMinVal, m_FrictionCurveMaximum, m_FrictionCurveMinimum);
            ApplyCurveValues(frictionCurveField, m_FrictionCurveMaximum, m_FrictionCurveMinimum);

            frictionCurveMaxField.RegisterCallback<ChangeEvent<Vector2>>(e =>
            {
                Vector2 curveMinVal = m_FrictionCurveMinimum.vector2Value;
                SanitizeCurveValues(e.newValue, curveMinVal, m_FrictionCurveMaximum, m_FrictionCurveMinimum);
                ApplyCurveValues(frictionCurveField, m_FrictionCurveMaximum, m_FrictionCurveMinimum);
            });
            frictionCurveMinField.RegisterCallback<ChangeEvent<Vector2>>(e =>
            {
                Vector2 curveMaxVal = m_FrictionCurveMaximum.vector2Value;
                SanitizeCurveValues(curveMaxVal, e.newValue, m_FrictionCurveMaximum, m_FrictionCurveMinimum);
                ApplyCurveValues(frictionCurveField, m_FrictionCurveMaximum, m_FrictionCurveMinimum);
            });

            container.Add(frictionCurveMaxField);
            container.Add(frictionCurveMinField);

            frictionCurveLeft.Add(curveYLabel);
            frictionCurveLeft.Add(corner);
            frictionCurveRight.Add(frictionCurveField);
            frictionCurveRight.Add(curveXLabel);
            frictionCurveContainer.Add(frictionCurveLeft);
            frictionCurveContainer.Add(frictionCurveRight);
            container.Add(frictionCurveContainer);

            return container;
        }

        static void SetInputFieldsDelayedInHierarchy(VisualElement root)
        {
            for (int i = 0; i < root.childCount; i++)
            {
                VisualElement elem = root.ElementAt(i);
                if (elem is TextInputBaseField<float>)
                {
                    (elem as TextInputBaseField<float>).isDelayed = true;
                }
                SetInputFieldsDelayedInHierarchy(elem);
            }
        }

        static void SanitizeCurveValues(Vector2 newMax, Vector2 newMin,
            SerializedProperty frictionCurveMaxProperty, SerializedProperty frictionCurveMinProperty)
        {
            newMax.x = math.clamp(newMax.x, MinCurveXIncrement, 1f - MinCurveXIncrement - MinCurveXIncrement);
            newMax.y = math.max(0, newMax.y);

            newMin.x = math.clamp(newMin.x, newMax.x + MinCurveXIncrement, 1f - MinCurveXIncrement);
            newMin.y = math.max(0, newMin.y);

            frictionCurveMaxProperty.vector2Value = newMax;
            frictionCurveMinProperty.vector2Value = newMin;
            frictionCurveMaxProperty.serializedObject.ApplyModifiedProperties();
            frictionCurveMinProperty.serializedObject.ApplyModifiedProperties();
        }

        static void ApplyCurveValues(CurveField frictionCurveField, SerializedProperty frictionCurveMaxProperty, SerializedProperty frictionCurveMinProperty)
        {
            Vector2 curveMaxVal = frictionCurveMaxProperty.vector2Value;
            Vector2 curveMinVal = frictionCurveMinProperty.vector2Value;

            frictionCurveField.value = new AnimationCurve
            {
                keys = new[]
                {
                    new Keyframe(0f, 0f),
                    new Keyframe(curveMaxVal.x, curveMaxVal.y),
                    new Keyframe(curveMinVal.x, curveMinVal.y),
                    new Keyframe(1f, curveMinVal.y),
                },
                postWrapMode = WrapMode.ClampForever
            }; ;

            SetLinearTangents(frictionCurveField);
        }

        static void SetLinearTangents(CurveField curveField)
        {
            var curve = curveField.value;
            Keyframe[] keys = curve.keys;

            for (int i = 0; i < keys.Length; i++)
            {
                float inSlope = 0;
                float outSlope = 0;

                if (i > 0)
                {
                    float dx = keys[i].time - keys[i - 1].time;
                    float dy = keys[i].value - keys[i - 1].value;
                    inSlope = dy / dx;
                }

                if (i < keys.Length - 1)
                {
                    float dx = keys[i + 1].time - keys[i].time;
                    float dy = keys[i + 1].value - keys[i].value;
                    outSlope = dy / dx;
                }

                keys[i] = new Keyframe(keys[i].time, keys[i].value, inSlope, outSlope);
            }

            curve.keys = keys;
            curveField.SetValueWithoutNotify(curve);
        }
    }
}