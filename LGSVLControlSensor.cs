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
using Simulator.Analysis;
using System.Collections;

namespace Simulator.Sensors
{
    [SensorType("LGSVL Control", new[] { typeof(VehicleControlData) })]
    public class LGSVLControlSensor : SensorBase, IVehicleInputs
    {
        VehicleControlData Data;
        IAgentController Controller;
        IVehicleDynamics Dynamics;

        double LastControlUpdate = 0f;
        float ActualLinVel = 0f;
        float ActualAngVel = 0f;

        public float SteerInput { get; private set; } = 0f;
        public float AccelInput { get; private set; } = 0f;
        public float BrakeInput { get; private set; } = 0f;
        public int? TargetGear { get; private set; } = 0;

        [AnalysisMeasurement(MeasurementType.Input)]
        public float MaxSteer = 0f;

        [AnalysisMeasurement(MeasurementType.Input)]
        public float MaxAccel = 0f;

        [AnalysisMeasurement(MeasurementType.Input)]
        public float MaxBrake = 0f;

        float ADAccelInput = 0f;
        float ADBrakeInput = 0f;
        float ADSteerInput = 0f;
        int ADTargetGear = 0;

        public AnimationCurve AccelerationInputCurve;
        public AnimationCurve BrakeInputCurve;

        VehicleControlData controlData;

        [SensorParameter]
        public bool UseNormalizedSteerAngle = false;

        [SensorParameter]
        public bool ResetWhenMissingInputs = true;

        [SensorParameter]
        public bool UseAutomaticTransmission = true;

        [SensorParameter]
        public float StuckTravelThreshold = 0.1f; // apollo autoware lgsvl sensor
        [SensorParameter]
        public float StuckTimeThreshold = 10.0f;
        private float ThrottleCommand = 0f;
        private float ThrottleCuttoff = 0.05f;
        private Vector3 StuckStartPosition;
        private float StuckTime;
        private bool EgoIsStuck = false;

        private void Awake()
        {
            LastControlUpdate = SimulatorManager.Instance.CurrentTime;
            Controller = GetComponentInParent<IAgentController>();
            Dynamics = GetComponentInParent<IVehicleDynamics>();
        }

        protected override void Initialize()
        {
            StuckStartPosition = transform.position;
        }

        protected override void Deinitialize()
        {

        }

        private void Update()
        {
            // stuck analysis
            ThrottleCommand = Dynamics.AccellInput;
            if (!EgoIsStuck && ThrottleCommand > ThrottleCuttoff && Vector3.Distance(transform.position, StuckStartPosition) < StuckTravelThreshold)
            {
                StuckTime += Time.fixedDeltaTime;
                if (StuckTime > StuckTimeThreshold)
                {
                    StuckEvent(Controller.GTID);
                    EgoIsStuck = true;
                }
            }
            else
            {
                StuckStartPosition = transform.position;
                StuckTime = 0f;
            }

            var projectedLinVec = Vector3.Project(Dynamics.Velocity, transform.forward);
            ActualLinVel = projectedLinVec.magnitude * (Vector3.Dot(Dynamics.Velocity, transform.forward) > 0 ? 1.0f : -1.0f);

            var projectedAngVec = Vector3.Project(Dynamics.AngularVelocity, transform.up);
            ActualAngVel = projectedAngVec.magnitude * (projectedAngVec.y > 0 ? -1.0f : 1.0f);

            if (ResetWhenMissingInputs)
            {
                // LastControlUpdate and Time.Time come from Unity.
                if (SimulatorManager.Instance.CurrentTime - LastControlUpdate >= 0.5)    // > 500ms
                {
                    ADAccelInput = ADSteerInput = AccelInput = SteerInput = 0f;
                }
            }
        }

        private void FixedUpdate()
        {
            if (SimulatorManager.Instance.CurrentTime - LastControlUpdate < 0.5f)
            {
                AccelInput = ADAccelInput;
                SteerInput = ADSteerInput;
                BrakeInput = ADBrakeInput;
                TargetGear = ADTargetGear;
                MaxSteer = Mathf.Max(MaxSteer, Mathf.Sign(SteerInput) * SteerInput);
                MaxAccel = Mathf.Max(MaxAccel, Mathf.Sign(AccelInput) * AccelInput);
                MaxBrake = Mathf.Max(MaxBrake, Mathf.Sign(BrakeInput) * BrakeInput);
            }
        }

        private float CalculateSteerInput(VehicleControlData data)
        {
            float wheelAngle = data.SteerAngle.GetValueOrDefault();
            if (UseNormalizedSteerAngle)
            {
                return Mathf.Clamp(wheelAngle, -1f, 1f);
            }
            else
            {
                wheelAngle = UnityEngine.Mathf.Clamp(wheelAngle, -Dynamics.MaxSteeringAngle, Dynamics.MaxSteeringAngle);
                var k = (float)(wheelAngle + Dynamics.MaxSteeringAngle) / (Dynamics.MaxSteeringAngle * 2);
                return UnityEngine.Mathf.Lerp(-1f, 1f, k);
            }
        }

        private void OnVehicleControlData(VehicleControlData data)
        {
            if (Time.timeScale == 0f)
                return;

            controlData = data;
            LastControlUpdate = SimulatorManager.Instance.CurrentTime;

            ADSteerInput = CalculateSteerInput(data);
            ADAccelInput = data.Acceleration.GetValueOrDefault();
            ADBrakeInput = data.Braking.GetValueOrDefault();
            ADTargetGear = (int)data.TargetGear.GetValueOrDefault();

            if (UseAutomaticTransmission)
            {
                if (ADTargetGear == (int)GearPosition.Reverse)
                {
                    Dynamics.ShiftReverseAutoGearBox();
                }
                else if (ADTargetGear == (int)GearPosition.Drive)
                {
                    Dynamics.ShiftFirstGear();
                }
            }
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            if (bridge.Plugin.GetBridgeNameAttribute().Type == "ROS")
            {
                bridge.AddSubscriber<VehicleControlData>(Topic, OnVehicleControlData);
            }
            else if (bridge.Plugin.GetBridgeNameAttribute().Type == "ROS2")
            {
                bridge.AddSubscriber<VehicleControlData>(Topic, OnVehicleControlData);
            }
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Assert(visualizer != null);
            var graphData = new Dictionary<string, object>()
            {
                {"AD Accel Input", ADAccelInput},
                {"AD Steer Input", ADSteerInput},
                {"AD Brake Input", ADBrakeInput},
                {"AD Target Gear", ADTargetGear},
                {"Last Control Update", LastControlUpdate},
                {"Actual Linear Velocity", ActualLinVel},
                {"Actual Angular Velocity", ActualAngVel},
                {"EgoIsStuck", EgoIsStuck },
            };

            if (controlData != null)
            {
                graphData.Add("Acceleration", controlData.Acceleration.GetValueOrDefault());
                graphData.Add("Braking", controlData.Braking.GetValueOrDefault());
                graphData.Add("Steer Angle", controlData.SteerAngle.GetValueOrDefault());
                graphData.Add("Target Gear", controlData.TargetGear.GetValueOrDefault());
            }

            visualizer.UpdateGraphValues(graphData);
        }

        public override void OnVisualizeToggle(bool state)
        {
            //
        }

        private void StuckEvent(uint id)
        {
            Hashtable data = new Hashtable
            {
                { "Id", id },
                { "Type", "Stuck" },
                { "Time", SimulatorManager.Instance.GetSessionElapsedTimeSpan().ToString() },
                { "Status", AnalysisManager.AnalysisStatusType.Failed },
            };
            SimulatorManager.Instance.AnalysisManager.AddEvent(data);
        }
    }
}
