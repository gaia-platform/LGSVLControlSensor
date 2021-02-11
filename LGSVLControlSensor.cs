/**
 * Copyright (c) 2019-2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Utilities;
using UnityEngine;
using Simulator.Sensors.UI;
using System.Collections.Generic;

namespace Simulator.Sensors
{
    [SensorType("LGSVL Control", new[] { typeof(VehicleControlData) })]
    public class LGSVLControlSensor : SensorBase, IVehicleInputs
    {
        VehicleControlData Data;
        VehicleController Controller;
        IVehicleDynamics Dynamics;

        double LastControlUpdate = 0f;
        float ActualLinVel = 0f;
        float ActualAngVel = 0f;

        public float SteerInput { get; private set; } = 0f;
        public float AccelInput { get; private set; } = 0f;
        public float BrakeInput { get; private set; } = 0f;

        [AnalysisMeasurement(MeasurementType.Input)]
        private float MaxSteer = 0f;

        [AnalysisMeasurement(MeasurementType.Input)]
        private float MaxAccel = 0f;

        [AnalysisMeasurement(MeasurementType.Input)]
        private float MaxBrake = 0f;

        float ADAccelInput = 0f;
        float ADSteerInput = 0f;

        public AnimationCurve AccelerationInputCurve;
        public AnimationCurve BrakeInputCurve;

        VehicleControlData controlData;


        private void Awake()
        {
            LastControlUpdate = SimulatorManager.Instance.CurrentTime;
            Controller = GetComponentInParent<VehicleController>();
            Dynamics = GetComponentInParent<IVehicleDynamics>();
        }

        private void Update()
        {
            var projectedLinVec = Vector3.Project(Dynamics.RB.velocity, transform.forward);
            ActualLinVel = projectedLinVec.magnitude * (Vector3.Dot(Dynamics.RB.velocity, transform.forward) > 0 ? 1.0f : -1.0f);

            var projectedAngVec = Vector3.Project(Dynamics.RB.angularVelocity, transform.up);
            ActualAngVel = projectedAngVec.magnitude * (projectedAngVec.y > 0 ? -1.0f : 1.0f);

            // LastControlUpdate and Time.Time come from Unity.
            if (SimulatorManager.Instance.CurrentTime - LastControlUpdate >= 0.5)    // > 500ms
            {
                ADAccelInput = ADSteerInput = AccelInput = SteerInput = 0f;
            }
        }

        private void FixedUpdate()
        {
            if (SimulatorManager.Instance.CurrentTime - LastControlUpdate < 0.5f)
            {
                AccelInput = ADAccelInput;
                SteerInput = ADSteerInput;
                MaxSteer = Mathf.Max(MaxSteer, Mathf.Sign(SteerInput) * SteerInput);
                MaxAccel = Mathf.Max(MaxAccel, Mathf.Sign(AccelInput) * AccelInput);
                MaxBrake = Mathf.Max(MaxBrake, Mathf.Sign(BrakeInput) * BrakeInput);
            }
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            bridge.AddSubscriber<VehicleControlData>(Topic, data =>
            {
                controlData = data;
                LastControlUpdate = SimulatorManager.Instance.CurrentTime;

                float wheelAngle = data.SteerAngle.GetValueOrDefault();
                wheelAngle = UnityEngine.Mathf.Clamp(wheelAngle, -Dynamics.MaxSteeringAngle, Dynamics.MaxSteeringAngle);
                var k = (float)(wheelAngle + Dynamics.MaxSteeringAngle) / (Dynamics.MaxSteeringAngle * 2);

                ADSteerInput = UnityEngine.Mathf.Lerp(-1f, 1f, k);
                ADAccelInput = data.Acceleration.GetValueOrDefault() - data.Braking.GetValueOrDefault();
            });
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Assert(visualizer != null);
            var graphData = new Dictionary<string, object>()
            {
                {"AD Accel Input", ADAccelInput},
                {"AD Steer Input", ADSteerInput},
                {"Last Control Update", LastControlUpdate},
                {"Actual Linear Velocity", ActualLinVel},
                {"Actual Angular Velocity", ActualAngVel},
            };

            if (controlData == null)
            {
                return;
            }
            graphData.Add("Acceleration", controlData.Acceleration.GetValueOrDefault());
            graphData.Add("Braking", controlData.Braking.GetValueOrDefault());
            graphData.Add("Steer Angle", controlData.SteerAngle.GetValueOrDefault());

            visualizer.UpdateGraphValues(graphData);
        }

        public override void OnVisualizeToggle(bool state)
        {
            //
        }
    }
}
