within NEUP;
package PowerPlant_Models
  model PowerPlant_test
    Reactor.NuScaleModule_v5 nuScaleModule_v5_1
      annotation (Placement(transformation(extent={{-64,-18},{-20,32}})));
    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.Data.Data_GenericModule
                                       data(length_steamGenerator_tube=36)
      annotation (Placement(transformation(extent={{62,82},{78,98}})));
    Modelica.Blocks.Sources.Constant const(k=0)
      annotation (Placement(transformation(extent={{-140,-12},{-120,8}})));
    BOP.NuScaleBOP_v1 nuScaleBOP_v1_1
      annotation (Placement(transformation(extent={{38,-18},{84,30}})));
  equation
    connect(const.y, nuScaleModule_v5_1.CR_reactivity) annotation (Line(points=
            {{-119,-2},{-96,-2},{-96,-1},{-73.68,-1}}, color={0,0,127}));
    connect(nuScaleBOP_v1_1.port_b, nuScaleModule_v5_1.port_b) annotation (Line(
          points={{40.3,-15.6},{12,-15.6},{12,-17},{-12.08,-17}}, color={0,127,
            255}));
    connect(nuScaleModule_v5_1.port_a, nuScaleBOP_v1_1.port_a) annotation (Line(
          points={{-12.96,14},{12,14},{12,27.12},{39.84,27.12}}, color={0,127,
            255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end PowerPlant_test;

  model PowerPlant_testv1
    Reactor.NuScaleModule_v5 nuScaleModule_v5_1
      annotation (Placement(transformation(extent={{-64,-18},{-20,32}})));
    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.Data.Data_GenericModule
                                       data(length_steamGenerator_tube=36)
      annotation (Placement(transformation(extent={{62,82},{78,98}})));
    BOP.NuScaleBOP_v2 nuScaleBOP_v2_1
      annotation (Placement(transformation(extent={{34,-20},{78,30}})));
    Modelica.Blocks.Interfaces.RealInput Reactivity
      annotation (Placement(transformation(extent={{-168,-22},{-128,18}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=nuScaleModule_v5_1.Core_Tavg)
      annotation (Placement(transformation(extent={{-100,-62},{-80,-42}})));
    Modelica.Blocks.Interfaces.RealOutput Avg_CoreTemp
      annotation (Placement(transformation(extent={{-60,-62},{-40,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=nuScaleModule_v5_1.Q_Total)
      annotation (Placement(transformation(extent={{-100,-82},{-80,-62}})));
    Modelica.Blocks.Interfaces.RealOutput Q_Core
      annotation (Placement(transformation(extent={{-62,-82},{-42,-62}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=nuScaleModule_v5_1.SG_outlet_Temp)
      annotation (Placement(transformation(extent={{-20,-64},{0,-44}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Temp
      annotation (Placement(transformation(extent={{20,-64},{40,-44}})));
    Modelica.Blocks.Sources.RealExpression realExpression3(y=nuScaleModule_v5_1.SG_outlet_Enthalpy)
      annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Enthalpy
      annotation (Placement(transformation(extent={{-60,-100},{-40,-80}})));
    Modelica.Blocks.Sources.RealExpression realExpression4(y=nuScaleBOP_v2_1.W_Sensor)
      annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
    Modelica.Blocks.Interfaces.RealOutput Sensor_W
      annotation (Placement(transformation(extent={{140,-60},{160,-40}})));
    Modelica.Blocks.Sources.RealExpression realExpression6(y=nuScaleBOP_v2_1.Q_LPT)
      annotation (Placement(transformation(extent={{100,-102},{120,-82}})));
    Modelica.Blocks.Interfaces.RealOutput LPT_Q
      annotation (Placement(transformation(extent={{140,-102},{160,-82}})));
    Modelica.Blocks.Sources.RealExpression realExpression5(y=nuScaleBOP_v2_1.Q_HPT)
      annotation (Placement(transformation(extent={{100,-80},{120,-60}})));
    Modelica.Blocks.Interfaces.RealOutput HPT_Q
      annotation (Placement(transformation(extent={{140,-80},{160,-60}})));
    Modelica.Blocks.Interfaces.RealInput FWP_input
      annotation (Placement(transformation(extent={{158,-28},{118,12}})));
  equation
    connect(nuScaleModule_v5_1.port_b, nuScaleBOP_v2_1.port_b) annotation (Line(
          points={{-12.08,-17},{12.96,-17},{12.96,-17},{13.76,-17}},    color={
            0,127,255}));
    connect(nuScaleModule_v5_1.port_a, nuScaleBOP_v2_1.port_a) annotation (Line(
          points={{-12.96,14},{12,14},{12,24},{12.88,24}}, color={0,127,255}));
    connect(Reactivity, nuScaleModule_v5_1.CR_reactivity) annotation (Line(
          points={{-148,-2},{-110,-2},{-110,-1},{-73.68,-1}}, color={0,0,127}));
    connect(realExpression.y, Avg_CoreTemp)
      annotation (Line(points={{-79,-52},{-50,-52}}, color={0,0,127}));
    connect(realExpression1.y, Q_Core)
      annotation (Line(points={{-79,-72},{-52,-72}}, color={0,0,127}));
    connect(realExpression2.y, SG_Outlet_Temp)
      annotation (Line(points={{1,-54},{30,-54}}, color={0,0,127}));
    connect(realExpression3.y, SG_Outlet_Enthalpy)
      annotation (Line(points={{-79,-90},{-50,-90}}, color={0,0,127}));
    connect(realExpression4.y, Sensor_W)
      annotation (Line(points={{121,-50},{150,-50}}, color={0,0,127}));
    connect(realExpression6.y, LPT_Q)
      annotation (Line(points={{121,-92},{150,-92}}, color={0,0,127}));
    connect(realExpression5.y, HPT_Q)
      annotation (Line(points={{121,-70},{150,-70}}, color={0,0,127}));
    connect(FWP_input, nuScaleBOP_v2_1.FW_Pump) annotation (Line(points={{138,
            -8},{94,-8},{94,-9.25},{49.62,-9.25}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=1500, __Dymola_Algorithm="Esdirk45a"));
  end PowerPlant_testv1;

  model INLm_PowerPlant_test
    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.Data.Data_GenericModule
                                       data(length_steamGenerator_tube=36)
      annotation (Placement(transformation(extent={{62,82},{78,98}})));
    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.SMR_Tave_enthalpy
                                 nuScale_Tave_enthalpy(
      redeclare NHES.Systems.PrimaryHeatSystem.SMR_Generic.CS_SMR_Tave_Enthalpy
        CS(
        T_SG_exit=579.15,
        Q_nom(displayUnit="MW") = 160000000,
        demand=1.0),
      redeclare NHES.Systems.PrimaryHeatSystem.SMR_Generic.ED_Dummy ED,
      port_a_nominal(
        m_flow=70,
        p=3447380,
        T(displayUnit="degC") = 421.15),
      port_b_nominal(
        p=3447380,
        T(displayUnit="degC") = 579.25,
        h=2997670))
      annotation (Placement(transformation(extent={{-100,-40},{-18,62}})));
    BOP.NuScaleBOP_v4_NoFWP nuScaleBOP_v4_NoFWP
      annotation (Placement(transformation(extent={{24,-28},{94,44}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=nuScaleBOP_v4_NoFWP.LPTurbine.Q_mech)
      annotation (Placement(transformation(extent={{140,-44},{160,-24}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=
          nuScale_Tave_enthalpy.core.Q_total.y)
      annotation (Placement(transformation(extent={{140,-72},{160,-52}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=
          nuScaleBOP_v4_NoFWP.HPTurbine.Q_mech)
      annotation (Placement(transformation(extent={{140,-18},{160,2}})));
    Modelica.Blocks.Sources.RealExpression realExpression3(y=
          nuScaleBOP_v4_NoFWP.sensorW.y)
      annotation (Placement(transformation(extent={{140,20},{160,40}})));
    Modelica.Blocks.Interfaces.RealOutput sensorW
      annotation (Placement(transformation(extent={{182,20},{202,40}})));
    Modelica.Blocks.Interfaces.RealOutput HPT_Q_mech
      annotation (Placement(transformation(extent={{182,-18},{202,2}})));
    Modelica.Blocks.Interfaces.RealOutput LPT_Q_mech
      annotation (Placement(transformation(extent={{182,-44},{202,-24}})));
    Modelica.Blocks.Interfaces.RealOutput Q_total_Core
      annotation (Placement(transformation(extent={{182,-72},{202,-52}})));
  equation
    connect(nuScaleBOP_v4_NoFWP.port_b, nuScale_Tave_enthalpy.port_a)
      annotation (Line(points={{27.5,-24.4},{9.6,-24.4},{9.6,4.72308},{-16.5091,
            4.72308}}, color={0,127,255}));
    connect(nuScaleBOP_v4_NoFWP.port_a, nuScale_Tave_enthalpy.port_b)
      annotation (Line(points={{26.8,39.68},{9.28,39.68},{9.28,30.6154},{
            -16.5091,30.6154}}, color={0,127,255}));
    connect(realExpression3.y, sensorW)
      annotation (Line(points={{161,30},{192,30}}, color={0,0,127}));
    connect(realExpression2.y, HPT_Q_mech)
      annotation (Line(points={{161,-8},{192,-8}}, color={0,0,127}));
    connect(realExpression.y, LPT_Q_mech)
      annotation (Line(points={{161,-34},{192,-34}}, color={0,0,127}));
    connect(realExpression1.y, Q_total_Core)
      annotation (Line(points={{161,-62},{192,-62}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end INLm_PowerPlant_test;

  model SMR_Test_v1
    extends Modelica.Icons.Example;

    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.SMR_Tave_enthalpy nuScale_Tave_enthalpy(
      redeclare NHES.Systems.PrimaryHeatSystem.SMR_Generic.CS_SMR_Tave_Enthalpy
        CS(
        T_SG_exit=579.15,
        Q_nom(displayUnit="MW") = 160000000,
        demand=1.0),
      port_a_nominal(
        m_flow=70,
        p=3447380,
        T(displayUnit="degC") = 421.15),
      port_b_nominal(
        p=3447380,
        T(displayUnit="degC") = 579.25,
        h=2997670))
      annotation (Placement(transformation(extent={{-76,-52},{20,64}})));
    BOP.NuScaleBOP_v3 nuScaleBOP_v3_1
      annotation (Placement(transformation(extent={{70,-34},{136,42}})));
  equation
    connect(nuScale_Tave_enthalpy.port_b, nuScaleBOP_v3_1.port_a) annotation (
        Line(points={{21.7455,28.3077},{46.8727,28.3077},{46.8727,37.44},{72.64,
            37.44}}, color={0,127,255}));
    connect(nuScale_Tave_enthalpy.port_a, nuScaleBOP_v3_1.port_b) annotation (
        Line(points={{21.7455,-1.13846},{47.8727,-1.13846},{47.8727,-30.2},{
            73.3,-30.2}}, color={0,127,255}));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},
              {100,100}})),
      experiment(
        StopTime=1800,
        __Dymola_NumberOfIntervals=180,
        __Dymola_Algorithm="Esdirk45a"),
      __Dymola_experimentSetupOutput(events=false),
      Icon(coordinateSystem(extent={{-100,-80},{100,100}})));
  end SMR_Test_v1;

  model PowerPlant_testv2
    Reactor.NuScaleModule_v5 nuScaleModule_v5_1
      annotation (Placement(transformation(extent={{-64,-18},{-20,32}})));
    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.Data.Data_GenericModule
                                       data(length_steamGenerator_tube=36)
      annotation (Placement(transformation(extent={{62,82},{78,98}})));
    BOP.NuScaleBOP_v2 nuScaleBOP_v2_1
      annotation (Placement(transformation(extent={{34,-20},{78,30}})));
    Modelica.Blocks.Interfaces.RealInput Reactivity
      annotation (Placement(transformation(extent={{-218,20},{-178,60}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=nuScaleModule_v5_1.Core_Tavg)
      annotation (Placement(transformation(extent={{-100,-62},{-80,-42}})));
    Modelica.Blocks.Interfaces.RealOutput Avg_CoreTemp
      annotation (Placement(transformation(extent={{-60,-62},{-40,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=nuScaleModule_v5_1.Q_Total)
      annotation (Placement(transformation(extent={{-100,-82},{-80,-62}})));
    Modelica.Blocks.Interfaces.RealOutput Q_Core
      annotation (Placement(transformation(extent={{-62,-82},{-42,-62}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=nuScaleModule_v5_1.SG_outlet_Temp)
      annotation (Placement(transformation(extent={{-20,-64},{0,-44}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Temp
      annotation (Placement(transformation(extent={{20,-64},{40,-44}})));
    Modelica.Blocks.Sources.RealExpression realExpression3(y=nuScaleModule_v5_1.SG_outlet_Enthalpy)
      annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Enthalpy
      annotation (Placement(transformation(extent={{-60,-100},{-40,-80}})));
    Modelica.Blocks.Sources.RealExpression realExpression4(y=nuScaleBOP_v2_1.W_Sensor)
      annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
    Modelica.Blocks.Interfaces.RealOutput Sensor_W
      annotation (Placement(transformation(extent={{140,-60},{160,-40}})));
    Modelica.Blocks.Sources.RealExpression realExpression6(y=nuScaleBOP_v2_1.Q_LPT)
      annotation (Placement(transformation(extent={{100,-102},{120,-82}})));
    Modelica.Blocks.Interfaces.RealOutput LPT_Q
      annotation (Placement(transformation(extent={{140,-102},{160,-82}})));
    Modelica.Blocks.Sources.RealExpression realExpression5(y=nuScaleBOP_v2_1.Q_HPT)
      annotation (Placement(transformation(extent={{100,-80},{120,-60}})));
    Modelica.Blocks.Interfaces.RealOutput HPT_Q
      annotation (Placement(transformation(extent={{140,-80},{160,-60}})));
    Modelica.Blocks.Interfaces.RealInput FWP_input
      annotation (Placement(transformation(extent={{206,6},{166,46}})));
    Modelica.Blocks.Sources.Constant const(k=0)
      annotation (Placement(transformation(extent={{-198,-12},{-178,8}})));
    Modelica.Blocks.Sources.Constant const1(k=70)
      annotation (Placement(transformation(extent={{194,-18},{174,2}})));
    Modelica.Blocks.Sources.RealExpression realExpression7(y=nuScaleBOP_v2_1.FWP_mflow.y)
      annotation (Placement(transformation(extent={{184,-62},{204,-42}})));
    Modelica.Blocks.Interfaces.RealOutput FWP_mflow
      annotation (Placement(transformation(extent={{224,-62},{244,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression8(y=nuScaleBOP_v2_1.Steam_mflow.y)
      annotation (Placement(transformation(extent={{186,-86},{206,-66}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Tsteam_mflow
      annotation (Placement(transformation(extent={{226,-86},{246,-66}})));
    Modelica.Blocks.Sources.RealExpression realExpression9(y=nuScaleModule_v5_1.P_mflow)
      annotation (Placement(transformation(extent={{-20,-86},{0,-66}})));
    Modelica.Blocks.Interfaces.RealOutput Primary_mflow
      annotation (Placement(transformation(extent={{20,-86},{40,-66}})));
    Modelica.Blocks.Sources.RealExpression realExpression10(y=
          nuScaleModule_v5_1.T_hotleg)
      annotation (Placement(transformation(extent={{-184,-60},{-164,-40}})));
    Modelica.Blocks.Interfaces.RealOutput T_hl
      annotation (Placement(transformation(extent={{-144,-60},{-124,-40}})));
    Modelica.Blocks.Sources.RealExpression realExpression11(y=
          nuScaleModule_v5_1.T_coldleg)
      annotation (Placement(transformation(extent={{-184,-82},{-164,-62}})));
    Modelica.Blocks.Interfaces.RealOutput T_cl
      annotation (Placement(transformation(extent={{-144,-82},{-124,-62}})));
    Modelica.Blocks.Sources.RealExpression realExpression12(y=
          nuScaleModule_v5_1.Secondary_Side_Pressure.p)
      annotation (Placement(transformation(extent={{-20,-104},{0,-84}})));
    Modelica.Blocks.Interfaces.RealOutput SG_press
      annotation (Placement(transformation(extent={{20,-104},{40,-84}})));
  equation
    connect(nuScaleModule_v5_1.port_b, nuScaleBOP_v2_1.port_b) annotation (Line(
          points={{-12.08,-17},{12.96,-17},{12.96,-17},{13.76,-17}},    color={
            0,127,255}));
    connect(nuScaleModule_v5_1.port_a, nuScaleBOP_v2_1.port_a) annotation (Line(
          points={{-12.96,14},{12,14},{12,24},{12.88,24}}, color={0,127,255}));
    connect(realExpression.y, Avg_CoreTemp)
      annotation (Line(points={{-79,-52},{-50,-52}}, color={0,0,127}));
    connect(realExpression1.y, Q_Core)
      annotation (Line(points={{-79,-72},{-52,-72}}, color={0,0,127}));
    connect(realExpression2.y, SG_Outlet_Temp)
      annotation (Line(points={{1,-54},{30,-54}}, color={0,0,127}));
    connect(realExpression3.y, SG_Outlet_Enthalpy)
      annotation (Line(points={{-79,-90},{-50,-90}}, color={0,0,127}));
    connect(realExpression4.y, Sensor_W)
      annotation (Line(points={{121,-50},{150,-50}}, color={0,0,127}));
    connect(realExpression6.y, LPT_Q)
      annotation (Line(points={{121,-92},{150,-92}}, color={0,0,127}));
    connect(realExpression5.y, HPT_Q)
      annotation (Line(points={{121,-70},{150,-70}}, color={0,0,127}));
    connect(realExpression7.y, FWP_mflow)
      annotation (Line(points={{205,-52},{234,-52}}, color={0,0,127}));
    connect(realExpression8.y, SG_Tsteam_mflow)
      annotation (Line(points={{207,-76},{236,-76}}, color={0,0,127}));
    connect(FWP_input, nuScaleBOP_v2_1.FW_Pump) annotation (Line(points={{186,
            26},{122,26},{122,-10},{56,-10},{56,-9.25},{49.62,-9.25}}, color={0,
            0,127}));
    connect(Reactivity, nuScaleModule_v5_1.CR_reactivity) annotation (Line(
          points={{-198,40},{-138,40},{-138,-1},{-73.68,-1}}, color={0,0,127}));
    connect(realExpression9.y, Primary_mflow)
      annotation (Line(points={{1,-76},{30,-76}}, color={0,0,127}));
    connect(realExpression10.y, T_hl)
      annotation (Line(points={{-163,-50},{-134,-50}}, color={0,0,127}));
    connect(realExpression11.y, T_cl)
      annotation (Line(points={{-163,-72},{-134,-72}}, color={0,0,127}));
    connect(realExpression12.y, SG_press)
      annotation (Line(points={{1,-94},{30,-94}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=1500, __Dymola_Algorithm="Esdirk45a"));
  end PowerPlant_testv2;

  model PowerPlant_testv3_valveadditionBOP
    Reactor.NuScaleModule_v5 nuScaleModule_v5_1
      annotation (Placement(transformation(extent={{-64,-18},{-20,32}})));
    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.Data.Data_GenericModule
                                       data(length_steamGenerator_tube=36)
      annotation (Placement(transformation(extent={{62,82},{78,98}})));
    Modelica.Blocks.Interfaces.RealInput Reactivity
      annotation (Placement(transformation(extent={{-286,64},{-246,104}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=nuScaleModule_v5_1.Core_Tavg)
      annotation (Placement(transformation(extent={{-100,-62},{-80,-42}})));
    Modelica.Blocks.Interfaces.RealOutput Avg_CoreTemp
      annotation (Placement(transformation(extent={{-60,-62},{-40,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=nuScaleModule_v5_1.Q_Total)
      annotation (Placement(transformation(extent={{-100,-82},{-80,-62}})));
    Modelica.Blocks.Interfaces.RealOutput Q_Core
      annotation (Placement(transformation(extent={{-62,-82},{-42,-62}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=nuScaleModule_v5_1.SG_outlet_Temp)
      annotation (Placement(transformation(extent={{-20,-64},{0,-44}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Temp
      annotation (Placement(transformation(extent={{20,-64},{40,-44}})));
    Modelica.Blocks.Sources.RealExpression realExpression3(y=nuScaleModule_v5_1.SG_outlet_Enthalpy)
      annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Enthalpy
      annotation (Placement(transformation(extent={{-60,-100},{-40,-80}})));
    Modelica.Blocks.Sources.RealExpression realExpression4(y=
          nuScaleBOP_v6_FWPdegrad.W_Sensor)
      annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
    Modelica.Blocks.Interfaces.RealOutput Sensor_W
      annotation (Placement(transformation(extent={{140,-60},{160,-40}})));
    Modelica.Blocks.Sources.RealExpression realExpression6(y=
          nuScaleBOP_v6_FWPdegrad.Q_LPT)
      annotation (Placement(transformation(extent={{100,-102},{120,-82}})));
    Modelica.Blocks.Interfaces.RealOutput LPT_Q
      annotation (Placement(transformation(extent={{140,-102},{160,-82}})));
    Modelica.Blocks.Sources.RealExpression realExpression5(y=
          nuScaleBOP_v6_FWPdegrad.Q_HPT)
      annotation (Placement(transformation(extent={{100,-80},{120,-60}})));
    Modelica.Blocks.Interfaces.RealOutput HPT_Q
      annotation (Placement(transformation(extent={{140,-80},{160,-60}})));
    Modelica.Blocks.Interfaces.RealInput FWP_input
      annotation (Placement(transformation(extent={{260,64},{228,96}}),
          iconTransformation(extent={{260,64},{228,96}})));
    Modelica.Blocks.Sources.Constant const(k=557.3)
      annotation (Placement(transformation(extent={{-186,-6},{-166,14}})));
    Modelica.Blocks.Sources.RealExpression realExpression7(y=
          nuScaleBOP_v6_FWPdegrad.FW_Pump)
      annotation (Placement(transformation(extent={{184,-62},{204,-42}})));
    Modelica.Blocks.Interfaces.RealOutput FWP_mflow
      annotation (Placement(transformation(extent={{224,-62},{244,-42}})));
    BOP.NuScaleBOP_v6_valveaddition nuScaleBOP_v6_FWPdegrad
      annotation (Placement(transformation(extent={{68,-22},{124,34}})));
    Modelica.Blocks.Sources.RealExpression realExpression10(y=
          nuScaleModule_v5_1.T_hotleg)
      annotation (Placement(transformation(extent={{-184,-62},{-164,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression11(y=
          nuScaleModule_v5_1.T_coldleg)
      annotation (Placement(transformation(extent={{-184,-84},{-164,-64}})));
    Modelica.Blocks.Interfaces.RealOutput T_hl
      annotation (Placement(transformation(extent={{-144,-62},{-124,-42}})));
    Modelica.Blocks.Interfaces.RealOutput T_cl
      annotation (Placement(transformation(extent={{-144,-84},{-124,-64}})));
    Modelica.Blocks.Sources.RealExpression realExpression9(y=nuScaleModule_v5_1.P_mflow)
      annotation (Placement(transformation(extent={{-20,-88},{0,-68}})));
    Modelica.Blocks.Interfaces.RealOutput Primary_mflow
      annotation (Placement(transformation(extent={{20,-88},{40,-68}})));
    Modelica.Blocks.Sources.RealExpression realExpression8(y=
          nuScaleBOP_v6_FWPdegrad.massFlowRate_steam.m_flow)
      annotation (Placement(transformation(extent={{186,-86},{206,-66}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Tsteam_mflow
      annotation (Placement(transformation(extent={{226,-86},{246,-66}})));
    Modelica.Blocks.Sources.Constant const1(k=87)
      annotation (Placement(transformation(extent={{292,36},{272,56}})));
    Modelica.Blocks.Sources.RealExpression realExpression12(y=
          nuScaleModule_v5_1.Secondary_Side_Pressure.p)
      annotation (Placement(transformation(extent={{-18,-104},{2,-84}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Pressure
      annotation (Placement(transformation(extent={{22,-104},{42,-84}})));
    TRANSFORM.Controls.P_Control p_Control(k=0.005)
      annotation (Placement(transformation(extent={{-144,42},{-124,62}})));
    Modelica.Blocks.Tables.CombiTable1Ds Valve_Position(table=[15,0.0051; 50,
          0.028; 75,0.092; 80,0.13; 85,0.16; 90,0.3; 95,0.8; 100,1])
      annotation (Placement(transformation(extent={{88,50},{68,70}})));
    Modelica.Blocks.Tables.CombiTable1Ds FWP_flowrate(table=[15,8; 50,26; 75,40;
          80,43; 85,45; 90,48; 95,51; 100,53])
      annotation (Placement(transformation(extent={{236,-34},{216,-14}})));
    Modelica.Blocks.Sources.TimeTable LF_sample(table=[0,100; 86400,100; 108000,
          100; 108900,80; 130500,80; 131400,100; 194400,100; 195300,80; 216900,
          80; 217800,100; 280800,100; 281700,80; 303300,80; 304200,100; 367200,
          100; 368100,80; 389700,80; 390600,100; 518400,100])
      annotation (Placement(transformation(extent={{292,-6},{272,14}})));
    Modelica.Blocks.Sources.TimeTable LF_profilev1(table=[0,100; 86400,100;
          108000,100; 108900,80; 130500,80; 131400,100; 194400,100; 195300,80;
          216900,80; 217800,100; 280800,100; 281700,80; 303300,80; 304200,100;
          367200,100; 368100,80; 389700,80; 390600,100; 518400,100; 0.0,0.0;
          0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,
          0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0;
          0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,0.0; 0.0,
          0.0; 0.0,0.0])
      annotation (Placement(transformation(extent={{294,-60},{274,-40}})));
    Modelica.Blocks.Sources.TimeTable LF_profilev2(table=[0,100; 86400,100;
          108000,100; 108900,80; 130500,80; 131400,100; 194400,100; 195300,80;
          216900,80; 217800,100; 280800,100; 281700,80; 303300,80; 304200,100;
          367200,100; 368100,80; 389700,80; 390600,100; 518400,100])
      annotation (Placement(transformation(extent={{294,-104},{274,-84}})));
  equation
    connect(realExpression.y, Avg_CoreTemp)
      annotation (Line(points={{-79,-52},{-50,-52}}, color={0,0,127}));
    connect(realExpression1.y, Q_Core)
      annotation (Line(points={{-79,-72},{-52,-72}}, color={0,0,127}));
    connect(realExpression2.y, SG_Outlet_Temp)
      annotation (Line(points={{1,-54},{30,-54}}, color={0,0,127}));
    connect(realExpression3.y, SG_Outlet_Enthalpy)
      annotation (Line(points={{-79,-90},{-50,-90}}, color={0,0,127}));
    connect(realExpression4.y, Sensor_W)
      annotation (Line(points={{121,-50},{150,-50}}, color={0,0,127}));
    connect(realExpression6.y, LPT_Q)
      annotation (Line(points={{121,-92},{150,-92}}, color={0,0,127}));
    connect(realExpression5.y, HPT_Q)
      annotation (Line(points={{121,-70},{150,-70}}, color={0,0,127}));
    connect(realExpression7.y, FWP_mflow)
      annotation (Line(points={{205,-52},{234,-52}}, color={0,0,127}));
    connect(nuScaleModule_v5_1.port_a, nuScaleBOP_v6_FWPdegrad.port_a)
      annotation (Line(points={{-12.96,14},{6,14},{6,27.28},{5.28,27.28}},
          color={0,127,255}));
    connect(nuScaleModule_v5_1.port_b, nuScaleBOP_v6_FWPdegrad.port_b)
      annotation (Line(points={{-12.08,-17},{-1.04,-17},{-1.04,-18.64},{42.24,
            -18.64}}, color={0,127,255}));
    connect(realExpression10.y,T_hl)
      annotation (Line(points={{-163,-52},{-134,-52}}, color={0,0,127}));
    connect(realExpression11.y,T_cl)
      annotation (Line(points={{-163,-74},{-134,-74}}, color={0,0,127}));
    connect(realExpression9.y,Primary_mflow)
      annotation (Line(points={{1,-78},{30,-78}}, color={0,0,127}));
    connect(realExpression8.y,SG_Tsteam_mflow)
      annotation (Line(points={{207,-76},{236,-76}}, color={0,0,127}));
    connect(realExpression12.y, SG_Pressure)
      annotation (Line(points={{3,-94},{32,-94}}, color={0,0,127}));
    connect(p_Control.y, nuScaleModule_v5_1.CR_reactivity) annotation (Line(
          points={{-123,52},{-123,28},{-73.68,28},{-73.68,-1}}, color={0,0,127}));
    connect(nuScaleModule_v5_1.Core_Tavg, p_Control.u_m) annotation (Line(
          points={{-89.3,-7.75},{-89.3,15.125},{-134,15.125},{-134,40}}, color=
            {0,0,127}));
    connect(const.y, p_Control.u_s) annotation (Line(points={{-165,4},{-156,4},
            {-156,52},{-146,52}}, color={0,0,127}));
    connect(SG_Pressure, SG_Pressure)
      annotation (Line(points={{32,-94},{32,-94}}, color={0,0,127}));
    connect(Valve_Position.y[1], nuScaleBOP_v6_FWPdegrad.T_valve) annotation (
        Line(points={{67,60},{48,60},{48,37.36},{28.8,37.36}}, color={0,0,127}));
    connect(FWP_flowrate.y[1], nuScaleBOP_v6_FWPdegrad.FW_Pump) annotation (
        Line(points={{215,-24},{130,-24},{130,-9.96},{87.88,-9.96}}, color={0,0,
            127}));
    connect(LF_sample.y, FWP_flowrate.u) annotation (Line(points={{271,4},{252,
            4},{252,-24},{238,-24}}, color={0,0,127}));
    connect(LF_sample.y, Valve_Position.u) annotation (Line(points={{271,4},{
            176,4},{176,60},{90,60}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(StopTime=5000, __Dymola_Algorithm="Esdirk45a"));
  end PowerPlant_testv3_valveadditionBOP;

  model PowerPlant_testv4_multivalvectrl
    Reactor.NuScaleModule_v5 nuScaleModule_v5_1
      annotation (Placement(transformation(extent={{-64,-18},{-20,32}})));
    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.Data.Data_GenericModule
                                       data(length_steamGenerator_tube=36)
      annotation (Placement(transformation(extent={{62,82},{78,98}})));
    Modelica.Blocks.Interfaces.RealInput Reactivity
      annotation (Placement(transformation(extent={{-286,64},{-246,104}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=nuScaleModule_v5_1.Core_Tavg)
      annotation (Placement(transformation(extent={{-100,-62},{-80,-42}})));
    Modelica.Blocks.Interfaces.RealOutput Avg_CoreTemp
      annotation (Placement(transformation(extent={{-60,-62},{-40,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=nuScaleModule_v5_1.Q_Total)
      annotation (Placement(transformation(extent={{-100,-82},{-80,-62}})));
    Modelica.Blocks.Interfaces.RealOutput Q_Core
      annotation (Placement(transformation(extent={{-62,-82},{-42,-62}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=nuScaleModule_v5_1.SG_outlet_Temp)
      annotation (Placement(transformation(extent={{-20,-64},{0,-44}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Temp
      annotation (Placement(transformation(extent={{20,-64},{40,-44}})));
    Modelica.Blocks.Sources.RealExpression realExpression3(y=nuScaleModule_v5_1.SG_outlet_Enthalpy)
      annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Enthalpy
      annotation (Placement(transformation(extent={{-60,-100},{-40,-80}})));
    Modelica.Blocks.Sources.RealExpression realExpression4(y=
          nuScaleBOP_v7_multivalvectrl.Sensor_W.y)
      annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
    Modelica.Blocks.Interfaces.RealOutput Sensor_W
      annotation (Placement(transformation(extent={{140,-60},{160,-40}})));
    Modelica.Blocks.Sources.RealExpression realExpression6(y=
          nuScaleBOP_v7_multivalvectrl.LPT_Qmech.y)
      annotation (Placement(transformation(extent={{100,-102},{120,-82}})));
    Modelica.Blocks.Interfaces.RealOutput LPT_Q
      annotation (Placement(transformation(extent={{140,-102},{160,-82}})));
    Modelica.Blocks.Sources.RealExpression realExpression5(y=
          nuScaleBOP_v7_multivalvectrl.HPT_Qmech.y)
      annotation (Placement(transformation(extent={{100,-80},{120,-60}})));
    Modelica.Blocks.Interfaces.RealOutput HPT_Q
      annotation (Placement(transformation(extent={{140,-80},{160,-60}})));
    Modelica.Blocks.Interfaces.RealInput FWP_input
      annotation (Placement(transformation(extent={{304,80},{272,112}}),
          iconTransformation(extent={{304,80},{272,112}})));
    Modelica.Blocks.Sources.Constant const(k=557.3)
      annotation (Placement(transformation(extent={{-186,-6},{-166,14}})));
    Modelica.Blocks.Sources.RealExpression realExpression7(y=
          nuScaleBOP_v7_multivalvectrl.massFlowRate_fwp.m_flow)
      annotation (Placement(transformation(extent={{184,-62},{204,-42}})));
    Modelica.Blocks.Interfaces.RealOutput FWP_mflow
      annotation (Placement(transformation(extent={{224,-62},{244,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression10(y=
          nuScaleModule_v5_1.T_hotleg)
      annotation (Placement(transformation(extent={{-184,-62},{-164,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression11(y=
          nuScaleModule_v5_1.T_coldleg)
      annotation (Placement(transformation(extent={{-184,-84},{-164,-64}})));
    Modelica.Blocks.Interfaces.RealOutput T_hl
      annotation (Placement(transformation(extent={{-144,-62},{-124,-42}})));
    Modelica.Blocks.Interfaces.RealOutput T_cl
      annotation (Placement(transformation(extent={{-144,-84},{-124,-64}})));
    Modelica.Blocks.Sources.RealExpression realExpression9(y=nuScaleModule_v5_1.P_mflow)
      annotation (Placement(transformation(extent={{-20,-88},{0,-68}})));
    Modelica.Blocks.Interfaces.RealOutput Primary_mflow
      annotation (Placement(transformation(extent={{20,-88},{40,-68}})));
    Modelica.Blocks.Sources.RealExpression realExpression8(y=
          nuScaleBOP_v7_multivalvectrl.Steam_mflow.y)
      annotation (Placement(transformation(extent={{186,-86},{206,-66}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Tsteam_mflow
      annotation (Placement(transformation(extent={{226,-86},{246,-66}})));
    Modelica.Blocks.Sources.Constant const1(k=3.39e+06)
      annotation (Placement(transformation(extent={{188,92},{174,106}})));
    Modelica.Blocks.Sources.RealExpression realExpression12(y=
          nuScaleModule_v5_1.Secondary_Side_Pressure.p)
      annotation (Placement(transformation(extent={{-18,-104},{2,-84}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Pressure
      annotation (Placement(transformation(extent={{22,-104},{42,-84}})));
    TRANSFORM.Controls.P_Control p_Control(k=0.005)
      annotation (Placement(transformation(extent={{-144,42},{-124,62}})));
    Modelica.Blocks.Tables.CombiTable1Ds FWP_flowrate(table=[15,8; 50,26; 75,40;
          80,43; 85,45; 90,48; 95,51; 100,53])
      annotation (Placement(transformation(extent={{198,-38},{178,-18}})));
    Modelica.Blocks.Sources.TimeTable LF_sample(table=[0,100; 86400,100; 108000,
          100; 108900,80; 130500,80; 131400,100; 194400,100; 195300,80; 216900,
          80; 217800,100; 280800,100; 281700,80; 303300,80; 304200,100; 367200,
          100; 368100,80; 389700,80; 390600,100; 518400,100])
      annotation (Placement(transformation(extent={{292,4},{272,24}})));
    Modelica.Blocks.Sources.TimeTable LF_profilev1(table=[0,100; 86400,100;
          108000,100; 109200,80; 130800,80; 132000,100; 150000,100; 150600,90;
          175800,90; 176400,100; 280800,100; 282000,80; 303600,80; 304800,100;
          363600,100; 364200,90; 375000,90; 375600,100; 453600,100; 454800,80;
          476400,80; 477600,100; 640800,100; 641400,90; 655800,90; 656400,100;
          712800,100; 714000,80; 735600,80; 736800,100; 885600,100; 886800,80;
          908400,80; 909600,100; 975600,100; 976200,90; 994200,90; 994800,100;
          1058400,100; 1059600,80; 1081200,80; 1082400,100; 1123200,100;
          1209600,100])
      annotation (Placement(transformation(extent={{292,-34},{272,-14}})));
    BOP.NuScaleBOP_v7_multivalvectrl nuScaleBOP_v7_multivalvectrl
      annotation (Placement(transformation(extent={{68,-18},{112,34}})));
    Modelica.Blocks.Sources.Ramp     ramp(
      height=-20,
      duration=1000,
      offset=100,
      startTime=350)
      annotation (Placement(transformation(extent={{294,40},{274,60}})));
    Modelica.Blocks.Logical.Switch switch1
      annotation (Placement(transformation(extent={{130,44},{114,60}})));
    Modelica.Blocks.Sources.ContinuousClock continuousClock(offset=0)
      annotation (Placement(transformation(extent={{252,60},{238,74}})));
    Modelica.Blocks.Logical.LessThreshold lessThreshold(threshold=300)
      annotation (Placement(transformation(extent={{196,46},{184,58}})));
    TRANSFORM.Controls.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      with_FF=false,
      k=-1,
      yb=1,
      k_s=1/const1.k,
      k_m=1/const1.k,
      yMax=1,
      yMin=0) annotation (Placement(transformation(extent={{86,54},{70,70}})));
  equation
    connect(realExpression.y, Avg_CoreTemp)
      annotation (Line(points={{-79,-52},{-50,-52}}, color={0,0,127}));
    connect(realExpression1.y, Q_Core)
      annotation (Line(points={{-79,-72},{-52,-72}}, color={0,0,127}));
    connect(realExpression2.y, SG_Outlet_Temp)
      annotation (Line(points={{1,-54},{30,-54}}, color={0,0,127}));
    connect(realExpression3.y, SG_Outlet_Enthalpy)
      annotation (Line(points={{-79,-90},{-50,-90}}, color={0,0,127}));
    connect(realExpression4.y, Sensor_W)
      annotation (Line(points={{121,-50},{150,-50}}, color={0,0,127}));
    connect(realExpression6.y, LPT_Q)
      annotation (Line(points={{121,-92},{150,-92}}, color={0,0,127}));
    connect(realExpression5.y, HPT_Q)
      annotation (Line(points={{121,-70},{150,-70}}, color={0,0,127}));
    connect(realExpression7.y, FWP_mflow)
      annotation (Line(points={{205,-52},{234,-52}}, color={0,0,127}));
    connect(realExpression10.y,T_hl)
      annotation (Line(points={{-163,-52},{-134,-52}}, color={0,0,127}));
    connect(realExpression11.y,T_cl)
      annotation (Line(points={{-163,-74},{-134,-74}}, color={0,0,127}));
    connect(realExpression9.y,Primary_mflow)
      annotation (Line(points={{1,-78},{30,-78}}, color={0,0,127}));
    connect(realExpression8.y,SG_Tsteam_mflow)
      annotation (Line(points={{207,-76},{236,-76}}, color={0,0,127}));
    connect(realExpression12.y, SG_Pressure)
      annotation (Line(points={{3,-94},{32,-94}}, color={0,0,127}));
    connect(p_Control.y, nuScaleModule_v5_1.CR_reactivity) annotation (Line(
          points={{-123,52},{-123,28},{-73.68,28},{-73.68,-1}}, color={0,0,127}));
    connect(nuScaleModule_v5_1.Core_Tavg, p_Control.u_m) annotation (Line(
          points={{-89.3,-7.75},{-89.3,15.125},{-134,15.125},{-134,40}}, color=
            {0,0,127}));
    connect(const.y, p_Control.u_s) annotation (Line(points={{-165,4},{-156,4},
            {-156,52},{-146,52}}, color={0,0,127}));
    connect(SG_Pressure, SG_Pressure)
      annotation (Line(points={{32,-94},{32,-94}}, color={0,0,127}));
    connect(nuScaleModule_v5_1.port_b, nuScaleBOP_v7_multivalvectrl.port_b)
      annotation (Line(points={{-12.08,-17},{17.96,-17},{17.96,-14.88},{47.76,
            -14.88}}, color={0,127,255}));
    connect(nuScaleModule_v5_1.port_a, nuScaleBOP_v7_multivalvectrl.port_a)
      annotation (Line(points={{-12.96,14},{4,14},{4,27.76},{18.72,27.76}},
          color={0,127,255}));
    connect(FWP_flowrate.y[1], nuScaleBOP_v7_multivalvectrl.FW_Pump)
      annotation (Line(points={{177,-28},{122,-28},{122,-6.82},{83.62,-6.82}},
          color={0,0,127}));
    connect(continuousClock.y, lessThreshold.u) annotation (Line(points={{237.3,
            67},{199.55,67},{199.55,52},{197.2,52}}, color={0,0,127}));
    connect(lessThreshold.y, switch1.u2)
      annotation (Line(points={{183.4,52},{131.6,52}}, color={255,0,255}));
    connect(switch1.y, PID.u_m) annotation (Line(points={{113.2,52},{102,52},{
            102,52.4},{78,52.4}}, color={0,0,127}));
    connect(nuScaleModule_v5_1.SG_press, switch1.u3) annotation (Line(points={{
            30.16,-8},{150,-8},{150,45.6},{131.6,45.6}}, color={0,0,127}));
    connect(const1.y, switch1.u1) annotation (Line(points={{173.3,99},{152.65,
            99},{152.65,58.4},{131.6,58.4}}, color={0,0,127}));
    connect(const1.y, PID.u_s) annotation (Line(points={{173.3,99},{105.65,99},
            {105.65,62},{87.6,62}}, color={0,0,127}));
    connect(PID.y, nuScaleBOP_v7_multivalvectrl.T_valve1) annotation (Line(
          points={{69.2,62},{46,62},{46,40.76},{20.92,40.76}}, color={0,0,127}));
    connect(LF_profilev1.y, FWP_flowrate.u) annotation (Line(points={{271,-24},
            {234,-24},{234,-28},{200,-28}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=1210000,
        Interval=100,
        Tolerance=1e-05,
        __Dymola_Algorithm="Esdirk45a"));
  end PowerPlant_testv4_multivalvectrl;

  model PowerPlant_test_DAVID_EXP
    Reactor.NuScaleModule_v5 nuScaleModule_v5_1
      annotation (Placement(transformation(extent={{-64,-18},{-20,32}})));
    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.Data.Data_GenericModule
                                       data(length_steamGenerator_tube=36)
      annotation (Placement(transformation(extent={{62,82},{78,98}})));
    Modelica.Blocks.Interfaces.RealInput Reactivity
      annotation (Placement(transformation(extent={{-286,64},{-246,104}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=nuScaleModule_v5_1.Core_Tavg)
      annotation (Placement(transformation(extent={{-100,-62},{-80,-42}})));
    Modelica.Blocks.Interfaces.RealOutput Avg_CoreTemp
      annotation (Placement(transformation(extent={{-60,-62},{-40,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=nuScaleModule_v5_1.Q_Total)
      annotation (Placement(transformation(extent={{-100,-82},{-80,-62}})));
    Modelica.Blocks.Interfaces.RealOutput Q_Core
      annotation (Placement(transformation(extent={{-62,-82},{-42,-62}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=nuScaleModule_v5_1.SG_outlet_Temp)
      annotation (Placement(transformation(extent={{-20,-64},{0,-44}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Temp
      annotation (Placement(transformation(extent={{20,-64},{40,-44}})));
    Modelica.Blocks.Sources.RealExpression realExpression3(y=nuScaleModule_v5_1.SG_outlet_Enthalpy)
      annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Enthalpy
      annotation (Placement(transformation(extent={{-60,-100},{-40,-80}})));
    Modelica.Blocks.Sources.RealExpression realExpression4(y=
          nuScaleBOP_v7_DAVID_EXP.Sensor_W.y)
      annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
    Modelica.Blocks.Interfaces.RealOutput Sensor_W
      annotation (Placement(transformation(extent={{140,-60},{160,-40}})));
    Modelica.Blocks.Sources.RealExpression realExpression6(y=
          nuScaleBOP_v7_DAVID_EXP.LPT_Qmech.y)
      annotation (Placement(transformation(extent={{100,-102},{120,-82}})));
    Modelica.Blocks.Interfaces.RealOutput LPT_Q
      annotation (Placement(transformation(extent={{140,-102},{160,-82}})));
    Modelica.Blocks.Sources.RealExpression realExpression5(y=
          nuScaleBOP_v7_DAVID_EXP.HPT_Qmech.y)
      annotation (Placement(transformation(extent={{100,-80},{120,-60}})));
    Modelica.Blocks.Interfaces.RealOutput HPT_Q
      annotation (Placement(transformation(extent={{140,-80},{160,-60}})));
    Modelica.Blocks.Interfaces.RealInput FWP_input
      annotation (Placement(transformation(extent={{304,80},{272,112}}),
          iconTransformation(extent={{304,80},{272,112}})));
    Modelica.Blocks.Sources.Constant const(k=557.3)
      annotation (Placement(transformation(extent={{-186,-6},{-166,14}})));
    Modelica.Blocks.Sources.RealExpression realExpression7(y=
          nuScaleBOP_v7_DAVID_EXP.massFlowRate_fwp.m_flow)
      annotation (Placement(transformation(extent={{184,-62},{204,-42}})));
    Modelica.Blocks.Interfaces.RealOutput FWP_mflow
      annotation (Placement(transformation(extent={{224,-62},{244,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression10(y=
          nuScaleModule_v5_1.T_hotleg)
      annotation (Placement(transformation(extent={{-184,-62},{-164,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression11(y=
          nuScaleModule_v5_1.T_coldleg)
      annotation (Placement(transformation(extent={{-184,-84},{-164,-64}})));
    Modelica.Blocks.Interfaces.RealOutput T_hl
      annotation (Placement(transformation(extent={{-144,-62},{-124,-42}})));
    Modelica.Blocks.Interfaces.RealOutput T_cl
      annotation (Placement(transformation(extent={{-144,-84},{-124,-64}})));
    Modelica.Blocks.Sources.RealExpression realExpression9(y=nuScaleModule_v5_1.P_mflow)
      annotation (Placement(transformation(extent={{-20,-88},{0,-68}})));
    Modelica.Blocks.Interfaces.RealOutput Primary_mflow
      annotation (Placement(transformation(extent={{20,-88},{40,-68}})));
    Modelica.Blocks.Sources.RealExpression realExpression8(y=
          nuScaleBOP_v7_DAVID_EXP.Steam_mflow.y)
      annotation (Placement(transformation(extent={{186,-86},{206,-66}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Tsteam_mflow
      annotation (Placement(transformation(extent={{226,-86},{246,-66}})));
    Modelica.Blocks.Sources.Constant const1(k=3.39e+06)
      annotation (Placement(transformation(extent={{188,92},{174,106}})));
    Modelica.Blocks.Sources.RealExpression realExpression12(y=
          nuScaleModule_v5_1.Secondary_Side_Pressure.p)
      annotation (Placement(transformation(extent={{-18,-104},{2,-84}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Pressure
      annotation (Placement(transformation(extent={{22,-104},{42,-84}})));
    TRANSFORM.Controls.P_Control p_Control(k=0.005)
      annotation (Placement(transformation(extent={{-144,42},{-124,62}})));
    Modelica.Blocks.Tables.CombiTable1Ds FWP_flowrate(table=[15,8; 50,26; 75,40;
          80,43; 85,45; 90,48; 95,51; 100,53])
      annotation (Placement(transformation(extent={{198,-38},{178,-18}})));
    Modelica.Blocks.Sources.TimeTable LF_sample(table=[0,100; 86400,100; 108000,
          100; 108900,80; 130500,80; 131400,100; 194400,100; 195300,80; 216900,
          80; 217800,100; 280800,100; 281700,80; 303300,80; 304200,100; 367200,
          100; 368100,80; 389700,80; 390600,100; 518400,100])
      annotation (Placement(transformation(extent={{292,4},{272,24}})));
    Modelica.Blocks.Sources.TimeTable LF_profilev1(table=[0,100; 86400,100;
          108000,100; 109200,80; 130800,80; 132000,100; 150000,100; 150600,90;
          175800,90; 176400,100; 280800,100; 282000,80; 303600,80; 304800,100;
          363600,100; 364200,90; 375000,90; 375600,100; 453600,100; 454800,80;
          476400,80; 477600,100; 640800,100; 641400,90; 655800,90; 656400,100;
          712800,100; 714000,80; 735600,80; 736800,100; 885600,100; 886800,80;
          908400,80; 909600,100; 975600,100; 976200,90; 994200,90; 994800,100;
          1058400,100; 1059600,80; 1081200,80; 1082400,100; 1123200,100;
          1209600,100])
      annotation (Placement(transformation(extent={{292,-34},{272,-14}})));
    Modelica.Blocks.Sources.Ramp     ramp(
      height=-20,
      duration=1000,
      offset=100,
      startTime=350)
      annotation (Placement(transformation(extent={{294,40},{274,60}})));
    Modelica.Blocks.Logical.Switch switch1
      annotation (Placement(transformation(extent={{130,44},{114,60}})));
    Modelica.Blocks.Sources.ContinuousClock continuousClock(offset=0)
      annotation (Placement(transformation(extent={{252,60},{238,74}})));
    Modelica.Blocks.Logical.LessThreshold lessThreshold(threshold=300)
      annotation (Placement(transformation(extent={{196,46},{184,58}})));
    TRANSFORM.Controls.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      with_FF=false,
      k=-1,
      yb=1,
      k_s=1/const1.k,
      k_m=1/const1.k,
      yMax=1,
      yMin=0) annotation (Placement(transformation(extent={{86,54},{70,70}})));
    BOP.NuScaleBOP_v7_DAVID_EXP nuScaleBOP_v7_DAVID_EXP
      annotation (Placement(transformation(extent={{76,-18},{122,32}})));
    Modelica.Blocks.Sources.Constant const2(k=100)
      annotation (Placement(transformation(extent={{240,2},{220,22}})));
  equation
    connect(realExpression.y, Avg_CoreTemp)
      annotation (Line(points={{-79,-52},{-50,-52}}, color={0,0,127}));
    connect(realExpression1.y, Q_Core)
      annotation (Line(points={{-79,-72},{-52,-72}}, color={0,0,127}));
    connect(realExpression2.y, SG_Outlet_Temp)
      annotation (Line(points={{1,-54},{30,-54}}, color={0,0,127}));
    connect(realExpression3.y, SG_Outlet_Enthalpy)
      annotation (Line(points={{-79,-90},{-50,-90}}, color={0,0,127}));
    connect(realExpression4.y, Sensor_W)
      annotation (Line(points={{121,-50},{150,-50}}, color={0,0,127}));
    connect(realExpression6.y, LPT_Q)
      annotation (Line(points={{121,-92},{150,-92}}, color={0,0,127}));
    connect(realExpression5.y, HPT_Q)
      annotation (Line(points={{121,-70},{150,-70}}, color={0,0,127}));
    connect(realExpression7.y, FWP_mflow)
      annotation (Line(points={{205,-52},{234,-52}}, color={0,0,127}));
    connect(realExpression10.y,T_hl)
      annotation (Line(points={{-163,-52},{-134,-52}}, color={0,0,127}));
    connect(realExpression11.y,T_cl)
      annotation (Line(points={{-163,-74},{-134,-74}}, color={0,0,127}));
    connect(realExpression9.y,Primary_mflow)
      annotation (Line(points={{1,-78},{30,-78}}, color={0,0,127}));
    connect(realExpression8.y,SG_Tsteam_mflow)
      annotation (Line(points={{207,-76},{236,-76}}, color={0,0,127}));
    connect(realExpression12.y, SG_Pressure)
      annotation (Line(points={{3,-94},{32,-94}}, color={0,0,127}));
    connect(p_Control.y, nuScaleModule_v5_1.CR_reactivity) annotation (Line(
          points={{-123,52},{-123,28},{-73.68,28},{-73.68,-1}}, color={0,0,127}));
    connect(nuScaleModule_v5_1.Core_Tavg, p_Control.u_m) annotation (Line(
          points={{-89.3,-7.75},{-89.3,15.125},{-134,15.125},{-134,40}}, color=
            {0,0,127}));
    connect(const.y, p_Control.u_s) annotation (Line(points={{-165,4},{-156,4},
            {-156,52},{-146,52}}, color={0,0,127}));
    connect(SG_Pressure, SG_Pressure)
      annotation (Line(points={{32,-94},{32,-94}}, color={0,0,127}));
    connect(continuousClock.y, lessThreshold.u) annotation (Line(points={{237.3,
            67},{199.55,67},{199.55,52},{197.2,52}}, color={0,0,127}));
    connect(lessThreshold.y, switch1.u2)
      annotation (Line(points={{183.4,52},{131.6,52}}, color={255,0,255}));
    connect(switch1.y, PID.u_m) annotation (Line(points={{113.2,52},{102,52},{
            102,52.4},{78,52.4}}, color={0,0,127}));
    connect(const1.y, switch1.u1) annotation (Line(points={{173.3,99},{152.65,
            99},{152.65,58.4},{131.6,58.4}}, color={0,0,127}));
    connect(const1.y, PID.u_s) annotation (Line(points={{173.3,99},{105.65,99},
            {105.65,62},{87.6,62}}, color={0,0,127}));
    connect(FWP_flowrate.y[1], nuScaleBOP_v7_DAVID_EXP.FW_Pump) annotation (
        Line(points={{177,-28},{92.33,-28},{92.33,-7.25}}, color={0,0,127}));
    connect(nuScaleBOP_v7_DAVID_EXP.T_valve1, PID.y) annotation (Line(points={{11.6,
            38.5},{11.6,62},{69.2,62}},        color={0,0,127}));
    connect(nuScaleModule_v5_1.port_a, nuScaleBOP_v7_DAVID_EXP.port_a)
      annotation (Line(points={{-12.96,14},{-12.96,32},{9.3,32},{9.3,26}},
          color={0,127,255}));
    connect(nuScaleModule_v5_1.port_b, nuScaleBOP_v7_DAVID_EXP.port_b)
      annotation (Line(points={{-12.08,-17},{-12.08,-26},{54.84,-26},{54.84,-15}},
          color={0,127,255}));
    connect(switch1.u3, nuScaleModule_v5_1.SG_press) annotation (Line(points={{
            131.6,45.6},{80.8,45.6},{80.8,-8},{30.16,-8}}, color={0,0,127}));
    connect(const2.y, FWP_flowrate.u) annotation (Line(points={{219,12},{210,12},
            {210,-28},{200,-28}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=7000,
        __Dymola_NumberOfIntervals=100,
        Tolerance=1e-05,
        __Dymola_Algorithm="Esdirk45a"));
  end PowerPlant_test_DAVID_EXP;

  model david_test
    TRANSFORM.Fluid.BoundaryConditions.FixedBoundary start(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p(displayUnit="Pa") = 3.39e+6,
      T=796.15,
      nPorts=1)
      annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
    TRANSFORM.Fluid.BoundaryConditions.FixedBoundary HPT(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=2768660,
      nPorts=1)
      annotation (Placement(transformation(extent={{100,-10},{80,10}})));
    TRANSFORM.Fluid.Machines.Pump_SimpleMassFlow pump(redeclare package Medium
        = Modelica.Media.Water.StandardWater, m_flow_nominal=67)
      annotation (Placement(transformation(extent={{-56,-10},{-36,10}})));
    TRANSFORM.Fluid.BoundaryConditions.FixedBoundary HPT1(
      redeclare package Medium = Modelica.Media.Water.StandardWater,
      p=2768660,
      nPorts=1) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={0,78})));
    TRANSFORM.Fluid.FittingsAndResistances.TeeJunctionVolume tee(redeclare
        package Medium = Modelica.Media.Water.StandardWater, V=1)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    TRANSFORM.Fluid.FittingsAndResistances.NominalLoss generic(
      dp_nominal(displayUnit="Pa") = 1e4,
      m_flow_nominal=0.001*67,
      p_nominal(displayUnit="Pa") = 1e6,
      T_nominal=726.15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,38})));
    TRANSFORM.Fluid.FittingsAndResistances.NominalLoss generic1
      annotation (Placement(transformation(extent={{-30,-58},{-10,-38}})));
    TRANSFORM.Fluid.FittingsAndResistances.MassTranportCoefficient resistance
      annotation (Placement(transformation(extent={{-86,-60},{-66,-40}})));
    TRANSFORM.Fluid.FittingsAndResistances.SpecifiedResistance resistance1
      annotation (Placement(transformation(extent={{34,-62},{54,-42}})));
  equation
    connect(start.ports[1], pump.port_a)
      annotation (Line(points={{-80,0},{-56,0}}, color={0,127,255}));
    connect(pump.port_b, tee.port_1)
      annotation (Line(points={{-36,0},{-10,0}}, color={0,127,255}));
    connect(tee.port_2, HPT.ports[1])
      annotation (Line(points={{10,0},{80,0}}, color={0,127,255}));
    connect(tee.port_3, generic.port_a)
      annotation (Line(points={{0,10},{-4.44089e-16,31}}, color={0,127,255}));
    connect(generic.port_b, HPT1.ports[1]) annotation (Line(points={{
            4.44089e-16,45},{-6.66134e-16,68}}, color={0,127,255}));
  end david_test;

  model PowerPlant_test_TCV_leak
    Reactor.NuScaleModule_v5 nuScaleModule_v5_1
      annotation (Placement(transformation(extent={{-64,-18},{-20,32}})));
    NHES.Systems.PrimaryHeatSystem.SMR_Generic.Components.Data.Data_GenericModule
                                       data(length_steamGenerator_tube=36)
      annotation (Placement(transformation(extent={{62,82},{78,98}})));
    Modelica.Blocks.Interfaces.RealInput Reactivity
      annotation (Placement(transformation(extent={{-286,64},{-246,104}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=nuScaleModule_v5_1.Core_Tavg)
      annotation (Placement(transformation(extent={{-100,-62},{-80,-42}})));
    Modelica.Blocks.Interfaces.RealOutput Avg_CoreTemp
      annotation (Placement(transformation(extent={{-60,-62},{-40,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=nuScaleModule_v5_1.Q_Total)
      annotation (Placement(transformation(extent={{-100,-82},{-80,-62}})));
    Modelica.Blocks.Interfaces.RealOutput Q_Core
      annotation (Placement(transformation(extent={{-62,-82},{-42,-62}})));
    Modelica.Blocks.Sources.RealExpression realExpression2(y=nuScaleModule_v5_1.SG_outlet_Temp)
      annotation (Placement(transformation(extent={{-20,-64},{0,-44}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Temp
      annotation (Placement(transformation(extent={{20,-64},{40,-44}})));
    Modelica.Blocks.Sources.RealExpression realExpression3(y=nuScaleModule_v5_1.SG_outlet_Enthalpy)
      annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Outlet_Enthalpy
      annotation (Placement(transformation(extent={{-60,-100},{-40,-80}})));
    Modelica.Blocks.Sources.RealExpression realExpression4(y=
          nuScaleBOP_v7_TCV_leak.Sensor_W.y)
      annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
    Modelica.Blocks.Interfaces.RealOutput Sensor_W
      annotation (Placement(transformation(extent={{140,-60},{160,-40}})));
    Modelica.Blocks.Sources.RealExpression realExpression6(y=
          nuScaleBOP_v7_TCV_leak.LPT_Qmech.y)
      annotation (Placement(transformation(extent={{100,-102},{120,-82}})));
    Modelica.Blocks.Interfaces.RealOutput LPT_Q
      annotation (Placement(transformation(extent={{140,-102},{160,-82}})));
    Modelica.Blocks.Sources.RealExpression realExpression5(y=
          nuScaleBOP_v7_TCV_leak.HPT_Qmech.y)
      annotation (Placement(transformation(extent={{100,-80},{120,-60}})));
    Modelica.Blocks.Interfaces.RealOutput HPT_Q
      annotation (Placement(transformation(extent={{140,-80},{160,-60}})));
    Modelica.Blocks.Interfaces.RealInput FWP_input
      annotation (Placement(transformation(extent={{304,80},{272,112}}),
          iconTransformation(extent={{304,80},{272,112}})));
    Modelica.Blocks.Sources.Constant const(k=557.3)
      annotation (Placement(transformation(extent={{-186,-6},{-166,14}})));
    Modelica.Blocks.Sources.RealExpression realExpression7(y=
          nuScaleBOP_v7_TCV_leak.massFlowRate_fwp.m_flow)
      annotation (Placement(transformation(extent={{184,-62},{204,-42}})));
    Modelica.Blocks.Interfaces.RealOutput FWP_mflow
      annotation (Placement(transformation(extent={{224,-62},{244,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression10(y=
          nuScaleModule_v5_1.T_hotleg)
      annotation (Placement(transformation(extent={{-184,-62},{-164,-42}})));
    Modelica.Blocks.Sources.RealExpression realExpression11(y=
          nuScaleModule_v5_1.T_coldleg)
      annotation (Placement(transformation(extent={{-184,-84},{-164,-64}})));
    Modelica.Blocks.Interfaces.RealOutput T_hl
      annotation (Placement(transformation(extent={{-144,-62},{-124,-42}})));
    Modelica.Blocks.Interfaces.RealOutput T_cl
      annotation (Placement(transformation(extent={{-144,-84},{-124,-64}})));
    Modelica.Blocks.Sources.RealExpression realExpression9(y=nuScaleModule_v5_1.P_mflow)
      annotation (Placement(transformation(extent={{-20,-88},{0,-68}})));
    Modelica.Blocks.Interfaces.RealOutput Primary_mflow
      annotation (Placement(transformation(extent={{20,-88},{40,-68}})));
    Modelica.Blocks.Sources.RealExpression realExpression8(y=
          nuScaleBOP_v7_TCV_leak.Steam_mflow.y)
      annotation (Placement(transformation(extent={{186,-86},{206,-66}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Tsteam_mflow
      annotation (Placement(transformation(extent={{226,-86},{246,-66}})));
    Modelica.Blocks.Sources.Constant const1(k=3.39e+06)
      annotation (Placement(transformation(extent={{188,92},{174,106}})));
    Modelica.Blocks.Sources.RealExpression realExpression12(y=
          nuScaleModule_v5_1.Secondary_Side_Pressure.p)
      annotation (Placement(transformation(extent={{-18,-104},{2,-84}})));
    Modelica.Blocks.Interfaces.RealOutput SG_Pressure
      annotation (Placement(transformation(extent={{22,-104},{42,-84}})));
    TRANSFORM.Controls.P_Control p_Control(k=0.005)
      annotation (Placement(transformation(extent={{-144,42},{-124,62}})));
    Modelica.Blocks.Tables.CombiTable1Ds FWP_flowrate(table=[15,8; 50,26; 75,40;
          80,43; 85,45; 90,48; 95,51; 100,53])
      annotation (Placement(transformation(extent={{198,-38},{178,-18}})));
    Modelica.Blocks.Sources.TimeTable LF_sample(table=[0,100; 86400,100; 108000,
          100; 108900,80; 130500,80; 131400,100; 194400,100; 195300,80; 216900,
          80; 217800,100; 280800,100; 281700,80; 303300,80; 304200,100; 367200,
          100; 368100,80; 389700,80; 390600,100; 518400,100])
      annotation (Placement(transformation(extent={{292,4},{272,24}})));
    Modelica.Blocks.Sources.TimeTable LF_profilev1(table=[0,100; 86400,100;
          108000,100; 109200,80; 130800,80; 132000,100; 150000,100; 150600,90;
          175800,90; 176400,100; 280800,100; 282000,80; 303600,80; 304800,100;
          363600,100; 364200,90; 375000,90; 375600,100; 453600,100; 454800,80;
          476400,80; 477600,100; 640800,100; 641400,90; 655800,90; 656400,100;
          712800,100; 714000,80; 735600,80; 736800,100; 885600,100; 886800,80;
          908400,80; 909600,100; 975600,100; 976200,90; 994200,90; 994800,100;
          1058400,100; 1059600,80; 1081200,80; 1082400,100; 1123200,100;
          1209600,100])
      annotation (Placement(transformation(extent={{292,-34},{272,-14}})));
    Modelica.Blocks.Sources.Ramp     ramp(
      height=-20,
      duration=1000,
      offset=100,
      startTime=350)
      annotation (Placement(transformation(extent={{294,40},{274,60}})));
    Modelica.Blocks.Logical.Switch switch1
      annotation (Placement(transformation(extent={{130,44},{114,60}})));
    Modelica.Blocks.Sources.ContinuousClock continuousClock(offset=0)
      annotation (Placement(transformation(extent={{252,60},{238,74}})));
    Modelica.Blocks.Logical.LessThreshold lessThreshold(threshold=300)
      annotation (Placement(transformation(extent={{196,46},{184,58}})));
    TRANSFORM.Controls.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      with_FF=false,
      k=-1,
      yb=1,
      k_s=1/const1.k,
      k_m=1/const1.k,
      yMax=1,
      yMin=0) annotation (Placement(transformation(extent={{86,54},{70,70}})));
    BOP.NuScaleBOP_v7_DAVID_EXP nuScaleBOP_v7_DAVID_EXP
      annotation (Placement(transformation(extent={{76,-18},{122,32}})));
    Modelica.Blocks.Sources.Constant const2(k=100)
      annotation (Placement(transformation(extent={{240,2},{220,22}})));
  equation
    connect(realExpression.y, Avg_CoreTemp)
      annotation (Line(points={{-79,-52},{-50,-52}}, color={0,0,127}));
    connect(realExpression1.y, Q_Core)
      annotation (Line(points={{-79,-72},{-52,-72}}, color={0,0,127}));
    connect(realExpression2.y, SG_Outlet_Temp)
      annotation (Line(points={{1,-54},{30,-54}}, color={0,0,127}));
    connect(realExpression3.y, SG_Outlet_Enthalpy)
      annotation (Line(points={{-79,-90},{-50,-90}}, color={0,0,127}));
    connect(realExpression4.y, Sensor_W)
      annotation (Line(points={{121,-50},{150,-50}}, color={0,0,127}));
    connect(realExpression6.y, LPT_Q)
      annotation (Line(points={{121,-92},{150,-92}}, color={0,0,127}));
    connect(realExpression5.y, HPT_Q)
      annotation (Line(points={{121,-70},{150,-70}}, color={0,0,127}));
    connect(realExpression7.y, FWP_mflow)
      annotation (Line(points={{205,-52},{234,-52}}, color={0,0,127}));
    connect(realExpression10.y,T_hl)
      annotation (Line(points={{-163,-52},{-134,-52}}, color={0,0,127}));
    connect(realExpression11.y,T_cl)
      annotation (Line(points={{-163,-74},{-134,-74}}, color={0,0,127}));
    connect(realExpression9.y,Primary_mflow)
      annotation (Line(points={{1,-78},{30,-78}}, color={0,0,127}));
    connect(realExpression8.y,SG_Tsteam_mflow)
      annotation (Line(points={{207,-76},{236,-76}}, color={0,0,127}));
    connect(realExpression12.y, SG_Pressure)
      annotation (Line(points={{3,-94},{32,-94}}, color={0,0,127}));
    connect(p_Control.y, nuScaleModule_v5_1.CR_reactivity) annotation (Line(
          points={{-123,52},{-123,28},{-73.68,28},{-73.68,-1}}, color={0,0,127}));
    connect(nuScaleModule_v5_1.Core_Tavg, p_Control.u_m) annotation (Line(
          points={{-89.3,-7.75},{-89.3,15.125},{-134,15.125},{-134,40}}, color=
            {0,0,127}));
    connect(const.y, p_Control.u_s) annotation (Line(points={{-165,4},{-156,4},
            {-156,52},{-146,52}}, color={0,0,127}));
    connect(SG_Pressure, SG_Pressure)
      annotation (Line(points={{32,-94},{32,-94}}, color={0,0,127}));
    connect(continuousClock.y, lessThreshold.u) annotation (Line(points={{237.3,
            67},{199.55,67},{199.55,52},{197.2,52}}, color={0,0,127}));
    connect(lessThreshold.y, switch1.u2)
      annotation (Line(points={{183.4,52},{131.6,52}}, color={255,0,255}));
    connect(switch1.y, PID.u_m) annotation (Line(points={{113.2,52},{102,52},{
            102,52.4},{78,52.4}}, color={0,0,127}));
    connect(const1.y, switch1.u1) annotation (Line(points={{173.3,99},{152.65,
            99},{152.65,58.4},{131.6,58.4}}, color={0,0,127}));
    connect(const1.y, PID.u_s) annotation (Line(points={{173.3,99},{105.65,99},
            {105.65,62},{87.6,62}}, color={0,0,127}));
    connect(FWP_flowrate.y[1], nuScaleBOP_v7_DAVID_EXP.FW_Pump) annotation (
        Line(points={{177,-28},{92.33,-28},{92.33,-7.25}}, color={0,0,127}));
    connect(nuScaleBOP_v7_DAVID_EXP.T_valve1, PID.y) annotation (Line(points={{11.6,
            38.5},{11.6,62},{69.2,62}},        color={0,0,127}));
    connect(nuScaleModule_v5_1.port_a, nuScaleBOP_v7_DAVID_EXP.port_a)
      annotation (Line(points={{-12.96,14},{-12.96,32},{9.3,32},{9.3,26}},
          color={0,127,255}));
    connect(nuScaleModule_v5_1.port_b, nuScaleBOP_v7_DAVID_EXP.port_b)
      annotation (Line(points={{-12.08,-17},{-12.08,-26},{54.84,-26},{54.84,-15}},
          color={0,127,255}));
    connect(switch1.u3, nuScaleModule_v5_1.SG_press) annotation (Line(points={{
            131.6,45.6},{80.8,45.6},{80.8,-8},{30.16,-8}}, color={0,0,127}));
    connect(const2.y, FWP_flowrate.u) annotation (Line(points={{219,12},{210,12},
            {210,-28},{200,-28}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=7000,
        __Dymola_NumberOfIntervals=100,
        Tolerance=1e-05,
        __Dymola_Algorithm="Esdirk45a"));
  end PowerPlant_test_TCV_leak;
end PowerPlant_Models;
