within NEUP.BOP;
model NuScaleBOP_v7_TCV_leak
  TRANSFORM.Fluid.Interfaces.FluidPort_Flow port_a(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-400,66},{-380,86}})));
  TRANSFORM.Fluid.Interfaces.FluidPort_State port_b(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-202,-98},{-182,-78}})));
  TRANSFORM.Fluid.Machines.SteamTurbine HPTurbine(redeclare package Medium =
        Modelica.Media.Water.StandardWater,
    p_a_start(displayUnit="MPa") = 3500000,
    p_b_start(displayUnit="MPa") = 1000000,
    use_T_start=false,
    h_a_start=3e6,
    m_flow_start=67,                        use_Stodola=false)
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  TRANSFORM.Fluid.Machines.SteamTurbine LPTurbine(redeclare package Medium =
        Modelica.Media.Water.StandardWater,
    p_a_start(displayUnit="MPa") = 1000000,
    p_b_start(displayUnit="kPa") = 10000,
    use_T_start=false,
    h_a_start=2.7e6,
    m_flow_start=(1 - 0.18)*67,             use_Stodola=false)
    annotation (Placement(transformation(extent={{58,60},{78,80}})));
  TRANSFORM.Fluid.Machines.Pump_SimpleMassFlow Cond_Pump(redeclare package
      Medium = Modelica.Media.Water.StandardWater, m_flow_nominal=(1 - 0.18)*67)
    annotation (Placement(transformation(extent={{22,-98},{2,-78}})));
  TRANSFORM.Fluid.Volumes.IdealCondenser condenser(redeclare package Medium =
        Modelica.Media.Water.StandardWater, p(displayUnit="kPa") = 10000)
    annotation (Placement(transformation(extent={{76,-98},{96,-78}})));
  Modelica.Fluid.Fittings.TeeJunctionVolume teeJunctionVolume(redeclare package
      Medium = Modelica.Media.Water.StandardWater,
    p_start(displayUnit="MPa") = 1000000,
    use_T_start=false,
    h_start=2.7e6,
    V=1)
    annotation (Placement(transformation(extent={{14,86},{-6,66}})));
  TRANSFORM.Fluid.FittingsAndResistances.NominalLoss Bleed(redeclare package
      Medium = Modelica.Media.Water.StandardWater,
    dp_nominal(displayUnit="Pa") = 1e4,
    m_flow_nominal=0.18*67,
    p_nominal(displayUnit="Pa") = 1e6,
    T_nominal=726.15)                              annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={14,6})));
  TRANSFORM.Fluid.Volumes.MixingVolume OFWH(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p_start(displayUnit="MPa") = 1000000,
    use_T_start=false,
    h_start=data.h_steam_cold,
    redeclare model Geometry =
        TRANSFORM.Fluid.ClosureRelations.Geometry.Models.LumpedVolume.GenericVolume
        (V=1),
    nPorts_b=2,
    nPorts_a=1)
    annotation (Placement(transformation(extent={{-38,-98},{-18,-78}})));
  TRANSFORM.Electrical.PowerConverters.Generator generator
    annotation (Placement(transformation(extent={{96,60},{116,80}})));
  TRANSFORM.Electrical.Sources.FrequencySource
                            boundary
    annotation (Placement(transformation(extent={{206,60},{186,80}})));
  TRANSFORM.Electrical.Sensors.PowerSensor sensorW
    annotation (Placement(transformation(extent={{140,60},{160,80}})));
  NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.Data.Data_GenericModule
                                     data(length_steamGenerator_tube=36)
    annotation (Placement(transformation(extent={{84,84},{100,100}})));
  TRANSFORM.Fluid.Machines.Pump_SimpleMassFlow Cond_Pump1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    use_input=true,
    m_flow_nominal=67)
    annotation (Placement(transformation(extent={{-48,-98},{-68,-78}})));
  Modelica.Blocks.Sources.RealExpression Sensor_W(y=sensorW.y)
    annotation (Placement(transformation(extent={{122,20},{142,40}})));
  Modelica.Blocks.Sources.RealExpression LPT_Qmech(y=LPTurbine.Q_mech)
    annotation (Placement(transformation(extent={{122,-6},{142,14}})));
  Modelica.Blocks.Sources.RealExpression HPT_Qmech(y=HPTurbine.Q_mech)
    annotation (Placement(transformation(extent={{122,-34},{142,-14}})));
  Modelica.Blocks.Interfaces.RealOutput W_Sensor
    annotation (Placement(transformation(extent={{170,20},{190,40}})));
  Modelica.Blocks.Interfaces.RealOutput Q_LPT
    annotation (Placement(transformation(extent={{170,-6},{190,14}})));
  Modelica.Blocks.Interfaces.RealOutput Q_HPT
    annotation (Placement(transformation(extent={{172,-34},{192,-14}})));
  Modelica.Blocks.Interfaces.RealInput FW_Pump annotation (Placement(
        transformation(extent={{-14,-72},{-44,-42}}), iconTransformation(extent=
           {{-14,-72},{-44,-42}})));
  Modelica.Fluid.Sensors.MassFlowRate massFlowRate_fwp(redeclare package Medium
      = Modelica.Media.Water.StandardWater, allowFlowReversal=false)
    annotation (Placement(transformation(extent={{-126,-96},{-146,-76}})));
  Modelica.Blocks.Sources.RealExpression FWP_mflow(y=massFlowRate_fwp.m_flow)
    annotation (Placement(transformation(extent={{122,-60},{142,-40}})));
  Modelica.Blocks.Interfaces.RealOutput mflow_feedwater
    annotation (Placement(transformation(extent={{172,-60},{192,-40}})));
  Modelica.Fluid.Sensors.MassFlowRate massFlowRate_steam(redeclare package
      Medium = Modelica.Media.Water.StandardWater, allowFlowReversal=false)
    annotation (Placement(transformation(extent={{-102,66},{-82,86}})));
  Modelica.Blocks.Sources.RealExpression Steam_mflow(y=massFlowRate_steam.m_flow)
    annotation (Placement(transformation(extent={{124,-88},{144,-68}})));
  Modelica.Blocks.Interfaces.RealOutput mflow_steam
    annotation (Placement(transformation(extent={{174,-88},{194,-68}})));
  Modelica.Fluid.Sensors.Pressure pressure_Tvalve(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-134,76},{-114,96}})));
  TRANSFORM.Fluid.Volumes.SimpleVolume volume(redeclare package Medium =
        Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-358,66},{-338,86}})));
  Modelica.Blocks.Interfaces.RealInput T_valve1 annotation (Placement(
        transformation(extent={{-390,116},{-370,136}}), iconTransformation(
          extent={{-390,116},{-370,136}})));
  TRANSFORM.Fluid.Valves.ValveCompressible TCV1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_start=100000,
    dp_nominal=100000,
    m_flow_nominal=67,
    p_nominal=2768700)
    annotation (Placement(transformation(extent={{-292,108},{-272,128}})));
  TRANSFORM.Fluid.Valves.ValveCompressible TCV2(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_nominal=100000,
    m_flow_nominal=67,
    p_nominal=2768700)
    annotation (Placement(transformation(extent={{-292,66},{-272,86}})));
  TRANSFORM.Fluid.Valves.ValveCompressible TCV3(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_nominal=100000,
    m_flow_nominal=67,
    p_nominal=2768700)
    annotation (Placement(transformation(extent={{-290,24},{-270,44}})));
  TRANSFORM.Fluid.FittingsAndResistances.NominalLoss TCV_leak(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_nominal(displayUnit="Pa") = 50,
    m_flow_nominal=0.005*53,
    p_nominal(displayUnit="Pa") = 3.39e6,
    T_nominal(displayUnit="K") = 576) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={-200,124})));
  Modelica.Fluid.Fittings.TeeJunctionVolume teeJunctionVolume1(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    p_start(displayUnit="Pa") = TCV1.p_nominal,
    use_T_start=false,
    V=1)
    annotation (Placement(transformation(extent={{-190,66},{-210,86}})));
  TRANSFORM.Fluid.BoundaryConditions.FixedBoundary atmosphere(redeclare package
      Medium = Modelica.Media.Water.StandardWater, nPorts=1)
    annotation (Placement(transformation(extent={{-116,116},{-136,136}})));
equation
  connect(LPTurbine.shaft_a, HPTurbine.shaft_b)
    annotation (Line(points={{58,70},{-40,70}}, color={0,0,0}));
  connect(LPTurbine.shaft_b, generator.shaft)
    annotation (Line(points={{78,70},{96,70}}, color={0,0,0}));
  connect(generator.port, sensorW.port_a)
    annotation (Line(points={{116,70},{140,70}}, color={255,0,0}));
  connect(sensorW.port_b, boundary.port)
    annotation (Line(points={{160,70},{186,70}}, color={255,0,0}));
  connect(HPTurbine.portLP, teeJunctionVolume.port_2)
    annotation (Line(points={{-40,76},{-6,76}}, color={0,127,255}));
  connect(teeJunctionVolume.port_1, LPTurbine.portHP)
    annotation (Line(points={{14,76},{58,76}}, color={0,127,255}));
  connect(LPTurbine.portLP, condenser.port_a)
    annotation (Line(points={{78,76},{79,76},{79,-81}}, color={0,127,255}));
  connect(condenser.port_b, Cond_Pump.port_a) annotation (Line(points={{86,-96},
          {56,-96},{56,-88},{22,-88}}, color={0,127,255}));
  connect(Cond_Pump.port_b, OFWH.port_b[1]) annotation (Line(points={{2,-88},{
          -10,-88},{-10,-88.5},{-22,-88.5}}, color={0,127,255}));
  connect(teeJunctionVolume.port_3, Bleed.port_a) annotation (Line(points={{4,66},{
          4,24},{14,24},{14,13}},      color={0,127,255}));
  connect(Bleed.port_b, OFWH.port_b[2]) annotation (Line(points={{14,-1},{-6,-1},
          {-6,-87.5},{-22,-87.5}}, color={0,127,255}));
  connect(Cond_Pump1.port_a, OFWH.port_a[1])
    annotation (Line(points={{-48,-88},{-34,-88}}, color={0,127,255}));
  connect(Sensor_W.y, W_Sensor)
    annotation (Line(points={{143,30},{180,30}}, color={0,0,127}));
  connect(LPT_Qmech.y, Q_LPT)
    annotation (Line(points={{143,4},{180,4}}, color={0,0,127}));
  connect(HPT_Qmech.y, Q_HPT)
    annotation (Line(points={{143,-24},{182,-24}}, color={0,0,127}));
  connect(Q_HPT, Q_HPT)
    annotation (Line(points={{182,-24},{182,-24}}, color={0,0,127}));
  connect(FW_Pump, Cond_Pump1.in_m_flow) annotation (Line(points={{-29,-57},{
          -43.5,-57},{-43.5,-80.7},{-58,-80.7}}, color={0,0,127}));
  connect(Cond_Pump1.port_b, massFlowRate_fwp.port_a) annotation (Line(points={
          {-68,-88},{-98,-88},{-98,-86},{-126,-86}}, color={0,127,255}));
  connect(massFlowRate_fwp.port_b, port_b) annotation (Line(points={{-146,-86},
          {-168,-86},{-168,-88},{-192,-88}}, color={0,127,255}));
  connect(FWP_mflow.y, mflow_feedwater)
    annotation (Line(points={{143,-50},{182,-50}}, color={0,0,127}));
  connect(mflow_feedwater, mflow_feedwater)
    annotation (Line(points={{182,-50},{182,-50}}, color={0,0,127}));
  connect(massFlowRate_steam.port_b, HPTurbine.portHP) annotation (Line(points={{-82,76},
          {-60,76}},                              color={0,127,255}));
  connect(Steam_mflow.y, mflow_steam)
    annotation (Line(points={{145,-78},{184,-78}}, color={0,0,127}));
  connect(mflow_steam, mflow_steam)
    annotation (Line(points={{184,-78},{184,-78}}, color={0,0,127}));
  connect(pressure_Tvalve.port, massFlowRate_steam.port_a)
    annotation (Line(points={{-124,76},{-102,76}}, color={0,127,255}));
  connect(port_a, volume.port_a)
    annotation (Line(points={{-390,76},{-354,76}}, color={0,127,255}));
  connect(TCV2.port_a, volume.port_b)
    annotation (Line(points={{-292,76},{-342,76}}, color={0,127,255}));
  connect(TCV1.port_a, volume.port_b) annotation (Line(points={{-292,118},{-316,
          118},{-316,76},{-342,76}}, color={0,127,255}));
  connect(TCV3.port_a, volume.port_b) annotation (Line(points={{-290,34},{-316,
          34},{-316,76},{-342,76}}, color={0,127,255}));
  connect(T_valve1, TCV1.opening)
    annotation (Line(points={{-380,126},{-282,126}}, color={0,0,127}));
  connect(T_valve1, TCV2.opening) annotation (Line(points={{-380,126},{-326,126},
          {-326,84},{-282,84}}, color={0,0,127}));
  connect(T_valve1, TCV3.opening) annotation (Line(points={{-380,126},{-326,126},
          {-326,42},{-280,42}}, color={0,0,127}));
  connect(TCV3.port_b, TCV2.port_b) annotation (Line(points={{-270,34},{-250,34},
          {-250,76},{-272,76}}, color={0,127,255}));
  connect(TCV1.port_b, TCV2.port_b) annotation (Line(points={{-272,118},{-250,
          118},{-250,76},{-272,76}}, color={0,127,255}));
  connect(teeJunctionVolume1.port_2, TCV2.port_b)
    annotation (Line(points={{-210,76},{-272,76}}, color={0,127,255}));
  connect(pressure_Tvalve.port, teeJunctionVolume1.port_1)
    annotation (Line(points={{-124,76},{-190,76}}, color={0,127,255}));
  connect(teeJunctionVolume1.port_3, TCV_leak.port_b)
    annotation (Line(points={{-200,86},{-200,117}}, color={0,127,255}));
  connect(TCV_leak.port_a, atmosphere.ports[1]) annotation (Line(points={{-200,
          131},{-168,131},{-168,126},{-136,126}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=1206000,
      __Dymola_NumberOfIntervals=100,
      __Dymola_Algorithm="Esdirk45a"));
end NuScaleBOP_v7_TCV_leak;
