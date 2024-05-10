within mtpPackage;

model RLTTypB1_old "Generic air-handling unit with heat recovery system"
  replaceable package Medium1 = Modelica.Media.Interfaces.PartialCondensingGases "Medium in air canal in the component" annotation(
    choices(choice(redeclare package Medium = AixLib.Media.Air "Moist air")));
  replaceable package Medium2 = Modelica.Media.Interfaces.PartialMedium "Medium in hydraulic circuits" annotation(
    choices(choice(redeclare package Medium = AixLib.Media.Air "Moist air"), choice(redeclare package Medium = AixLib.Media.Water "Water"), choice(redeclare package Medium = AixLib.Media.Antifreeze.PropyleneGlycolWater(property_T = 293.15, X_a = 0.40) "Propylene glycol water, 40% mass fraction")));
  parameter Modelica.Units.SI.Time tau = 15 "Time Constant for PT1 behavior of temperature sensors" annotation(
    Dialog(tab = "Advanced"));
  parameter Modelica.Fluid.Types.Dynamics energyDynamics = Modelica.Fluid.Types.Dynamics.DynamicFreeInitial "Type of energy balance: dynamic (3 initialization options) or steady state" annotation(
    Evaluate = true,
    Dialog(tab = "Dynamics", group = "Equations"));
  parameter Modelica.Fluid.Types.Dynamics massDynamics = Modelica.Fluid.Types.Dynamics.DynamicFreeInitial "Type of mass balance: dynamic (3 initialization options) or steady state" annotation(
    Evaluate = true,
    Dialog(tab = "Dynamics", group = "Equations"));
  parameter AixLib.Fluid.Movers.BaseClasses.Characteristics.efficiencyParameters hydraulicEfficiency(V_flow = {0}, eta = {0.7}) "Hydraulic efficiency of the fans" annotation(
    Dialog(group = "Fans"));
  parameter Modelica.Units.SI.Temperature T_amb = 293.15 "Ambient temperature";
  parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal = 1 "Nominal mass flow rate in air canal";
  parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal = 0.5 "Nominal mass flow rate in hydraulics";
  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal_kvs = 0.1 "Nominal mass flow rate in kvs";
  //-----parameters-----
  parameter Modelica.Units.SI.Volume vol = 0.0005 "Mixing Volume" annotation(
    Dialog(tab = "Advanced"));
  parameter AixLib.Fluid.Actuators.Valves.Data.GenericThreeWay valveCharacteristic "Valve characteristic of three way valve" annotation(
    choicesAllMatching = true,
    Placement(transformation(extent = {{-120, -120}, {-100, -100}})),
    Dialog(group = "Actuators"));
  parameter Real Kv = 10;
  parameter Modelica.Units.SI.Temperature T_start = 303.15 "Initialization temperature" annotation(
    Dialog(tab = "Advanced", group = "Initialization"));
  parameter Boolean usePreheater = true "If true, a preaheater is used" annotation(
    choices(checkBox = true),
    Dialog(group = "Preheater"));
  parameter Boolean useHumidifierRet = true "If true, a humidifier in return canal is used" annotation(
    choices(checkBox = true),
    Dialog(group = "Humidifiers"));
  parameter Boolean useHumidifier = true "If true, a humidifier in supply canal is used" annotation(
    choices(checkBox = true),
    Dialog(group = "Humidifiers"));
  parameter Boolean allowFlowReversal1 = true "= false to simplify equations, assuming, but not enforcing, no flow reversal for medium in air canal" annotation(
    Dialog(tab = "Assumptions"),
    Evaluate = true);
  parameter Boolean allowFlowReversal2 = true "= false to simplify equations, assuming, but not enforcing, no flow reversal for medium in hydraulics" annotation(
    Dialog(tab = "Assumptions"),
    Evaluate = true);
  Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium = Medium1, h_outflow(start = Medium1.h_default, nominal = Medium1.h_default)) "Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)" annotation(
    Placement(transformation(extent = {{-230, -10}, {-210, 10}}), iconTransformation(extent = {{-230, -10}, {-210, 10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(redeclare package Medium = Medium1, h_outflow(start = Medium1.h_default, nominal = Medium1.h_default)) "Fluid connector b1 (positive design flow direction is from port_a1 to port_b1)" annotation(
    Placement(transformation(extent = {{230, -10}, {210, 10}}), iconTransformation(extent = {{232, -10}, {212, 10}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a2(redeclare package Medium = Medium1, h_outflow(start = Medium1.h_default, nominal = Medium1.h_default)) "Fluid connector a2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(transformation(extent = {{210, 70}, {230, 90}}), iconTransformation(extent = {{212, 70}, {232, 90}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2(redeclare package Medium = Medium1, h_outflow(start = Medium1.h_default, nominal = Medium1.h_default)) "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(transformation(extent = {{-210, 70}, {-230, 90}}), iconTransformation(extent = {{-210, 70}, {-230, 90}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a3(redeclare package Medium = Medium2) if usePreheater "Fluid connector a3 (positive design flow direction is from port_a3 to port_b3)" annotation(
    Placement(visible = true, transformation(origin = {146, 0}, extent = {{-170, -110}, {-150, -90}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-170, -110}, {-150, -90}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b3(redeclare package Medium = Medium2) if usePreheater "Fluid connector b3 (positive design flow direction is from port_a3 to port_b3)" annotation(
    Placement(visible = true, transformation(origin = {76, 0}, extent = {{-130, -110}, {-110, -90}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-130, -110}, {-110, -90}}, rotation = 0)));
  AixLib.Fluid.Actuators.Dampers.Exponential flapOda(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, final dpDamper_nominal = dpDamper_nominal) "Supply air flap" annotation(
    Placement(transformation(extent = {{-190, -10}, {-170, 10}})));
  AixLib.Fluid.Actuators.Dampers.Exponential flapRet(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, final dpDamper_nominal = dpDamper_nominal) "Return air flap" annotation(
    Placement(transformation(extent = {{200, 70}, {180, 90}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a4(redeclare package Medium = Medium2) "Fluid connector a2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {46, 0}, extent = {{-10, -110}, {10, -90}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -110}, {10, -90}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b4(redeclare package Medium = Medium2) "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {-24, 0}, extent = {{30, -110}, {50, -90}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{30, -110}, {50, -90}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_a port_a5(redeclare package Medium = Medium2) "Fluid connector a2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {26, 0}, extent = {{70, -110}, {90, -90}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{70, -110}, {90, -90}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b5(redeclare package Medium = Medium2) "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {-44, 0}, extent = {{110, -110}, {130, -90}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{108, -110}, {128, -90}}, rotation = 0)));
  AixLib.Fluid.Movers.FlowControlled_dp fanSup(redeclare package Medium = Medium1, energyDynamics = energyDynamics, T_start = T_start, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, redeclare AixLib.Fluid.Movers.Data.Generic per(hydraulicEfficiency = hydraulicEfficiency, motorEfficiency(eta = {0.95})), final inputType = AixLib.Fluid.Types.InputType.Continuous) "Supply air fan" annotation(
    Placement(visible = true, transformation(origin = {-46, 0}, extent = {{156, -10}, {176, 10}}, rotation = 0)));
  AixLib.Fluid.Movers.FlowControlled_dp fanRet(redeclare package Medium = Medium1, energyDynamics = energyDynamics, T_start = T_start, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, redeclare AixLib.Fluid.Movers.Data.Generic per(hydraulicEfficiency = hydraulicEfficiency, motorEfficiency(eta = {0.95})), final inputType = AixLib.Fluid.Types.InputType.Continuous) "Return air fan" annotation(
    Placement(visible = true, transformation(origin = {90, 80}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
  AixLib.Fluid.Humidifiers.GenericHumidifier_u humidifier(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, energyDynamics = energyDynamics, T_start = T_start) if useHumidifier "Steam or adiabatic humdifier in supply canal" annotation(
    Placement(visible = true, transformation(origin = {-80, 0}, extent = {{130, -10}, {150, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senTRet(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, T_start = T_start) "Return air temperature sensor" annotation(
    Placement(transformation(extent = {{160, 70}, {140, 90}})));
  AixLib.Fluid.Sensors.TemperatureTwoPort senTExh(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, T_start = T_start) "Exhaust air temperature sensor" annotation(
    Placement(transformation(extent = {{-140, 70}, {-160, 90}})));
  AixLib.Fluid.Sensors.TemperatureTwoPort senTSup(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, T_start = T_start) "Supply air temperature sensor" annotation(
    Placement(visible = true, transformation(origin = {160, 0}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  AixLib.Fluid.Sensors.VolumeFlowRate senVolFlo(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, m_flow_nominal = m1_flow_nominal) "Exhaust volume flow sensor" annotation(
    Placement(visible = true, transformation(origin = {-110, 80}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
  AixLib.Fluid.Sensors.RelativeHumidityTwoPort senRelHumSup(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, m_flow_nominal = m1_flow_nominal) "Supply air humidity sensor" annotation(
    Placement(visible = true, transformation(origin = {-46, 0}, extent = {{180, -6}, {192, 6}}, rotation = 0)));
  parameter Modelica.Units.SI.PressureDifference dpDamper_nominal = 1 "Pressure drop of fully open dampers at nominal mass flow rate";
  AixLib.Fluid.HeatExchangers.DynamicHX cooler(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, Q_nom = 1000*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {30, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.HeatExchangers.DynamicHX reHeater(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, Q_nom = 1000*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {90, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.HeatExchangers.DynamicHX preHeater(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, Q_nom = 1000*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {-10, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  mtpPackage.GenericAHUBus_extended genericAHUBus_extended annotation(
    Placement(visible = true, transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 154}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  AixLib.Fluid.Actuators.Dampers.Exponential flapEha(redeclare package Medium = Medium1, allowFlowReversal = allowFlowReversal1, dpDamper_nominal = dpDamper_nominal, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {-370, 0}, extent = {{200, 70}, {180, 90}}, rotation = 0)));
  AixLib.Fluid.Actuators.Dampers.Exponential flapSup(redeclare package Medium = Medium1, allowFlowReversal = allowFlowReversal1, dpDamper_nominal = dpDamper_nominal, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {370, 0}, extent = {{-190, -10}, {-170, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senTOda(redeclare package Medium = Medium1, T_start = T_start, allowFlowReversal = allowFlowReversal1, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {-204, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.HeatExchangers.DynamicHX kvsEha(Q_nom = 1000*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {-70, 74}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.HeatExchangers.DynamicHX kvsSup(Q_nom = 1000*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {-70, 6}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.Actuators.Valves.ThreeWayTable mixingValve(redeclare package Medium = Medium2, CvData = AixLib.Fluid.Types.CvTypes.Kv, Kv = Kv, T_start = T_start, dpFixed_nominal = {10, 10}, energyDynamics = energyDynamics, flowCharacteristics1 = valveCharacteristic.a_ab, flowCharacteristics3 = valveCharacteristic.b_ab, init = Modelica.Blocks.Types.Init.InitialState, m_flow_nominal = m_flow_nominal_kvs, order = 1, portFlowDirection_1 = Modelica.Fluid.Types.PortFlowDirection.Entering, portFlowDirection_2 = Modelica.Fluid.Types.PortFlowDirection.Leaving, portFlowDirection_3 = Modelica.Fluid.Types.PortFlowDirection.Entering, tau = 0.2, y_start = 0) annotation(
    Placement(visible = true, transformation(origin = {-92, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  replaceable AixLib.Systems.HydraulicModules.BaseClasses.BasicPumpInterface PumpInterface(redeclare package Medium = Medium2, final allowFlowReversal = false, final m_flow_nominal = m_flow_nominal_kvs, T_start = T_start, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {-72, 2}, extent = {{42, 12}, {58, 28}}, rotation = 90)));
  AixLib.Fluid.MixingVolumes.MixingVolume junc(redeclare package Medium = Medium2, final massDynamics = massDynamics, T_start = T_start, nPorts = 3, final m_flow_nominal = m_flow_nominal_kvs, final V = vol, final energyDynamics = energyDynamics) annotation(
    Placement(visible = true, transformation(origin = {-38, 30}, extent = {{-10, 10}, {10, -10}}, rotation = 90)));
protected
  Modelica.Blocks.Continuous.FirstOrder PT1_airIn(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(visible = true, transformation(origin = {160, 20}, extent = {{4, -4}, {-4, 4}}, rotation = 270)));
  Modelica.Blocks.Continuous.FirstOrder PT1_airIn1(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) "Oda sensor delay" annotation(
    Placement(transformation(extent = {{4, -4}, {-4, 4}}, rotation = 270, origin = {-206, 26})));
  Modelica.Blocks.Continuous.FirstOrder PT1_airIn2(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(transformation(extent = {{4, -4}, {-4, 4}}, rotation = 270, origin = {-150, 106})));
  Modelica.Blocks.Continuous.FirstOrder PT1_airIn3(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(transformation(extent = {{4, -4}, {-4, 4}}, rotation = 270, origin = {150, 102})));
equation
  connect(senTSup.T, PT1_airIn.u) annotation(
    Line(points = {{160, 9}, {160, 15}}, color = {0, 0, 127}));
  connect(senTExh.T, PT1_airIn2.u) annotation(
    Line(points = {{-150, 91}, {-150, 101.2}}, color = {0, 0, 127}));
  connect(senTRet.T, PT1_airIn3.u) annotation(
    Line(points = {{150, 91}, {150, 97.2}}, color = {0, 0, 127}));
  connect(preHeater.port_b1, cooler.port_a1) annotation(
    Line(points = {{0, 0}, {20, 0}}, color = {0, 127, 255}));
  connect(preHeater.port_b2, port_b3) annotation(
    Line(points = {{-20, -12}, {-44, -12}, {-44, -100}}, color = {0, 127, 255}));
  connect(preHeater.port_a2, port_a3) annotation(
    Line(points = {{0, -12}, {0, -50}, {-14, -50}, {-14, -100}}, color = {0, 127, 255}));
  connect(cooler.port_b2, port_b4) annotation(
    Line(points = {{20, -12}, {16, -12}, {16, -100}}, color = {0, 127, 255}));
  connect(cooler.port_a2, port_a4) annotation(
    Line(points = {{40, -12}, {46, -12}, {46, -100}}, color = {0, 127, 255}));
  connect(reHeater.port_a2, port_a5) annotation(
    Line(points = {{100, -12}, {106, -12}, {106, -100}}, color = {0, 127, 255}));
  connect(reHeater.port_b2, port_b5) annotation(
    Line(points = {{80, -12}, {76, -12}, {76, -100}}, color = {0, 127, 255}));
  connect(PT1_airIn1.y, genericAHUBus_extended.TOdaMea) annotation(
    Line(points = {{-206, 30}, {-206, 52}, {-242, 52}, {-242, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(flapOda.y, genericAHUBus_extended.flapOdaSet) annotation(
    Line(points = {{-180, 12}, {-180, 52}, {-242, 52}, {-242, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(flapOda.y_actual, genericAHUBus_extended.flapOdaMea) annotation(
    Line(points = {{-174, 8}, {-174, 52}, {-242, 52}, {-242, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(flapEha.y_actual, genericAHUBus_extended.flapEhaMea) annotation(
    Line(points = {{-184, 88}, {-184, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(flapEha.y, genericAHUBus_extended.flapEhaSet) annotation(
    Line(points = {{-180, 92}, {-180, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(PT1_airIn2.y, genericAHUBus_extended.TEhaMea) annotation(
    Line(points = {{-150, 110}, {-150, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(senVolFlo.V_flow, genericAHUBus_extended.V_flow_EtaMea) annotation(
    Line(points = {{-110, 92}, {-110, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(fanRet.dp_in, genericAHUBus_extended.dpFanEtaSet) annotation(
    Line(points = {{90, 92}, {90, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(fanRet.P, genericAHUBus_extended.powerFanRetMea) annotation(
    Line(points = {{80, 90}, {80, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(fanRet.dp_actual, genericAHUBus_extended.dpFanEtaMea) annotation(
    Line(points = {{80, 86}, {72, 86}, {72, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(PT1_airIn3.y, genericAHUBus_extended.TEtaMea) annotation(
    Line(points = {{150, 106}, {150, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(flapRet.y, genericAHUBus_extended.flapEtaSet) annotation(
    Line(points = {{190, 92}, {190, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(flapRet.y_actual, genericAHUBus_extended.flapEtaMea) annotation(
    Line(points = {{186, 88}, {186, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(fanSup.dp_in, genericAHUBus_extended.dpFanSupSet) annotation(
    Line(points = {{120, 12}, {120, 52}, {234, 52}, {234, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(fanSup.P, genericAHUBus_extended.powerFanSupMea) annotation(
    Line(points = {{132, 10}, {130, 10}, {130, 52}, {234, 52}, {234, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(fanSup.dp_actual, genericAHUBus_extended.dpFanSupMea) annotation(
    Line(points = {{132, 6}, {140, 6}, {140, 52}, {234, 52}, {234, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(senRelHumSup.phi, genericAHUBus_extended.relHumSupMea) annotation(
    Line(points = {{140, 6}, {140, 52}, {234, 52}, {234, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(PT1_airIn.y, genericAHUBus_extended.TSupMea) annotation(
    Line(points = {{160, 24}, {160, 52}, {234, 52}, {234, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(flapSup.y, genericAHUBus_extended.flapSupSet) annotation(
    Line(points = {{190, 12}, {190, 52}, {234, 52}, {234, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(flapSup.y_actual, genericAHUBus_extended.flapSupMea) annotation(
    Line(points = {{196, 8}, {196, 52}, {234, 52}, {234, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(humidifier.powerEva, genericAHUBus_extended.powerSteamHumMea) annotation(
    Line(points = {{72, 10}, {88, 10}, {88, 52}, {234, 52}, {234, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(cooler.port_b1, humidifier.port_a) annotation(
    Line(points = {{40, 0}, {50, 0}}, color = {0, 127, 255}));
  connect(humidifier.port_b, reHeater.port_a1) annotation(
    Line(points = {{70, 0}, {80, 0}}, color = {0, 127, 255}));
  connect(reHeater.port_b1, fanSup.port_a) annotation(
    Line(points = {{100, 0}, {110, 0}}, color = {0, 127, 255}));
  connect(fanSup.port_b, senRelHumSup.port_a) annotation(
    Line(points = {{130, 0}, {134, 0}}, color = {0, 127, 255}));
  connect(senRelHumSup.port_b, senTSup.port_a) annotation(
    Line(points = {{146, 0}, {152, 0}}, color = {0, 127, 255}));
  connect(senTSup.port_b, flapSup.port_a) annotation(
    Line(points = {{168, 0}, {180, 0}}, color = {0, 127, 255}));
  connect(flapSup.port_b, port_b1) annotation(
    Line(points = {{200, 0}, {220, 0}}, color = {0, 127, 255}));
  connect(senTOda.T, PT1_airIn1.u) annotation(
    Line(points = {{-204, 11}, {-204, 21.2}, {-206, 21.2}}, color = {0, 0, 127}));
  connect(senTOda.port_b, flapOda.port_a) annotation(
    Line(points = {{-194, 0}, {-190, 0}}, color = {0, 127, 255}));
  connect(port_a1, senTOda.port_a) annotation(
    Line(points = {{-220, 0}, {-214, 0}}, color = {0, 127, 255}));
  connect(port_a2, flapRet.port_a) annotation(
    Line(points = {{220, 80}, {200, 80}}));
  connect(flapRet.port_b, senTRet.port_a) annotation(
    Line(points = {{180, 80}, {160, 80}}, color = {0, 127, 255}));
  connect(senTRet.port_b, fanRet.port_a) annotation(
    Line(points = {{140, 80}, {100, 80}}, color = {0, 127, 255}));
  connect(senVolFlo.port_b, senTExh.port_a) annotation(
    Line(points = {{-120, 80}, {-140, 80}}, color = {0, 127, 255}));
  connect(senTExh.port_b, flapEha.port_a) annotation(
    Line(points = {{-160, 80}, {-170, 80}}, color = {0, 127, 255}));
  connect(flapEha.port_b, port_b2) annotation(
    Line(points = {{-190, 80}, {-220, 80}}, color = {0, 127, 255}));
  connect(humidifier.u, genericAHUBus_extended.humSetSup) annotation(
    Line(points = {{50, 6}, {48, 6}, {48, 52}, {234, 52}, {234, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(flapOda.port_b, kvsSup.port_a1) annotation(
    Line(points = {{-170, 0}, {-80, 0}}, color = {0, 127, 255}));
  connect(kvsSup.port_b1, preHeater.port_a1) annotation(
    Line(points = {{-60, 0}, {-20, 0}}, color = {0, 127, 255}));
  connect(senVolFlo.port_a, kvsEha.port_b1) annotation(
    Line(points = {{-100, 80}, {-80, 80}}, color = {0, 127, 255}));
  connect(kvsEha.port_a1, fanRet.port_b) annotation(
    Line(points = {{-60, 80}, {80, 80}}, color = {0, 127, 255}));
  connect(mixingValve.port_1, kvsSup.port_b2) annotation(
    Line(points = {{-92, 20}, {-92, 12}, {-80, 12}}, color = {0, 127, 255}));
  connect(PumpInterface.port_a, mixingValve.port_2) annotation(
    Line(points = {{-92, 44}, {-92, 40}}, color = {0, 127, 255}));
  connect(PumpInterface.port_b, kvsEha.port_a2) annotation(
    Line(points = {{-92, 60}, {-92, 68}, {-80, 68}}, color = {0, 127, 255}));
  connect(junc.ports[1], kvsSup.port_a2) annotation(
    Line(points = {{-48, 30}, {-48, 12}, {-60, 12}}, color = {0, 127, 255}));
  connect(mixingValve.port_3, junc.ports[2]) annotation(
    Line(points = {{-82, 30}, {-48, 30}}, color = {0, 127, 255}));
  connect(kvsEha.port_b2, junc.ports[3]) annotation(
    Line(points = {{-60, 68}, {-48, 68}, {-48, 30}}, color = {0, 127, 255}));
  connect(mixingValve.y, genericAHUBus_extended.kvsValveSet) annotation(
    Line(points = {{-104, 30}, {-162, 30}, {-162, 52}, {-242, 52}, {-242, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(mixingValve.y_actual, genericAHUBus_extended.kvsValveMea) annotation(
    Line(points = {{-98, 36}, {-98, 52}, {-242, 52}, {-242, 120}, {0, 120}}, color = {0, 0, 127}));
  connect(genericAHUBus_extended.kvsPumpBus, PumpInterface.pumpBus) annotation(
    Line(points = {{0, 120}, {-92, 120}, {-92, 70}, {-100, 70}, {-100, 52}}, color = {255, 204, 51}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-220, -100}, {220, 120}}), graphics = {Rectangle(fillColor = {255, 255, 255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-220, 120}, {220, -100}}), Line(points = {{-210, 0}, {-166, 0}, {-90, 0}}, color = {28, 108, 200}), Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-164, 38}, {-116, -40}}), Rectangle(extent = {{-4, 38}, {44, -40}}), Rectangle(extent = {{74, 38}, {122, -40}}), Rectangle(extent = {{-90, 100}, {-30, -40}}), Line(points = {{-164, -40}, {-116, 38}}), Line(points = {{-4, -40}, {44, 38}}), Line(points = {{74, -40}, {122, 38}}), Line(points = {{-4, 36}, {44, -40}}), Line(points = {{-90, -34}, {-36, 100}}), Line(points = {{-84, -40}, {-30, 94}}), Line(points = {{-90, 100}, {-30, -40}}), Line(points = {{122, 0}, {166, 0}}, color = {28, 108, 200}), Line(points = {{144, 80}, {-30, 80}}, color = {28, 108, 200}), Ellipse(extent = {{202, -18}, {166, 18}}), Line(points = {{176, 16}, {200, 8}}), Line(points = {{200, -8}, {176, -16}}), Ellipse(origin = {162, 80}, rotation = 180, extent = {{18, -18}, {-18, 18}}), Line(origin = {158, 68}, rotation = 180, points = {{-12, 4}, {12, -4}}), Line(origin = {158, 92}, rotation = 180, points = {{12, 4}, {-12, -4}}), Line(points = {{212, 80}, {180, 80}}, color = {28, 108, 200}), Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{138, 38}, {160, -40}}), Line(points = {{146, 24}, {152, 28}}), Line(points = {{152, 24}, {146, 24}}), Line(points = {{152, 20}, {146, 24}}), Line(points = {{146, 0}, {152, 4}}), Line(points = {{152, 0}, {146, 0}}), Line(points = {{152, -4}, {146, 0}}), Line(points = {{146, -20}, {152, -16}}), Line(points = {{152, -20}, {146, -20}}), Line(points = {{152, -24}, {146, -20}}), Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{0, 100}, {20, 58}}), Line(points = {{8, 86}, {14, 90}}), Line(points = {{14, 90}, {8, 90}}), Line(points = {{14, 90}, {8, 94}}), Line(points = {{8, 66}, {14, 70}}), Line(points = {{14, 70}, {8, 70}}), Line(points = {{14, 70}, {8, 74}}), Line(points = {{0, 78}}, color = {28, 108, 200}), Line(points = {{-90, 80}, {-210, 80}}, color = {28, 108, 200}), Line(points = {{-30, 0}, {-4, 0}}, color = {28, 108, 200}), Line(points = {{44, 0}, {74, 0}}, color = {28, 108, 200}), Line(points = {{202, 0}, {218, 0}}, color = {28, 108, 200}), Line(points = {{-160, -40}, {-160, -90}}, color = {28, 108, 200}), Line(points = {{-120, -40}, {-120, -90}}, color = {28, 108, 200}), Line(points = {{0, -40}, {0, -90}}, color = {28, 108, 200}), Line(points = {{40, -40}, {40, -90}}, color = {28, 108, 200}), Line(points = {{80, -40}, {80, -90}}, color = {28, 108, 200}), Line(points = {{118, -40}, {118, -90}}, color = {28, 108, 200})}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-220, -100}, {220, 120}})),
    Documentation(revisions = "<html>
<ul>
<li>October 29, 2019, by Alexander K&uuml;mpel:<br/>First implementation</li>
</ul>
</html>", info = "<html>
<p>The GenericAHU is an air-handling unit model with detailed hydraulic system of the preheater, heater and cooler. The ahu includes a heat exchanger for heat recovery and a humidifier for the supply air. The humidifier can be insert steam or water that evaporates completely (adiabatic). Further, the ahu includes an adiabatic humidifier in the return air chanal in order to cool the return air and use the heat recovery heat excahnger to cool the supply air. The preheater, steam humdifier and adiabatic humidifier are conditional and can be deactivated.</p>
</html>"));
end RLTTypB1_old;
