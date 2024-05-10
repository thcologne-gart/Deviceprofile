within mtpPackage;

model RLTTypB2_withBoundary "Generic air-handling unit with heat recovery system"
  replaceable package Medium1 = AixLib.Media.Air "Medium in air canal in the component" annotation(
    choices(choice(redeclare package Medium = AixLib.Media.Air "Moist air")));
  replaceable package Medium2 = AixLib.Media.Water "Medium 1 in the component" annotation(
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
  parameter Modelica.Units.SI.Volume vol = 0.0005 "Mixing Volume" annotation(
    Dialog(tab = "Advanced"));
  parameter AixLib.Fluid.Actuators.Valves.Data.Generic datVal(phi = {0, 0.19, 0.35, 0.45, 0.5, 0.65}/0.65, y = {0, 0.1667, 0.3333, 0.5, 0.6667, 1}) annotation(
    Placement(visible = true, transformation(origin = {-300, 0}, extent = {{60, 60}, {80, 80}}, rotation = 0)));
  parameter Real Kv = 10;
  parameter Modelica.Units.SI.Temperature T_amb = 293.15 "Ambient temperature";
  parameter Modelica.Units.SI.MassFlowRate m1_flow_nominal = 1 "Nominal mass flow rate in air canal";
  parameter Modelica.Units.SI.MassFlowRate m2_flow_nominal = 0.5 "Nominal mass flow rate in hydraulics";
  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal_kvs = 0.1 "Nominal mass flow rate in kvs";
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
    Placement(visible = true, transformation(origin = {-80, -40}, extent = {{-230, -10}, {-210, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-230, -10}, {-210, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(redeclare package Medium = Medium1, h_outflow(start = Medium1.h_default, nominal = Medium1.h_default)) "Fluid connector b1 (positive design flow direction is from port_a1 to port_b1)" annotation(
    Placement(visible = true, transformation(origin = {80, -40}, extent = {{230, -10}, {210, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{232, -10}, {212, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_a port_a2(redeclare package Medium = Medium1, h_outflow(start = Medium1.h_default, nominal = Medium1.h_default)) "Fluid connector a2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {80, 60}, extent = {{210, 70}, {230, 90}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{212, 70}, {232, 90}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2(redeclare package Medium = Medium1, h_outflow(start = Medium1.h_default, nominal = Medium1.h_default)) "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {-80, 60}, extent = {{-210, 70}, {-230, 90}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-210, 70}, {-230, 90}}, rotation = 0)));
  AixLib.Fluid.Actuators.Dampers.Exponential flapOda(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, final dpDamper_nominal = dpDamper_nominal) "Supply air flap" annotation(
    Placement(visible = true, transformation(origin = {-90, -40}, extent = {{-190, -10}, {-170, 10}}, rotation = 0)));
  AixLib.Fluid.FixedResistances.PressureDrop filterOda(redeclare package Medium = Medium1, dp_nominal = 10, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {-230, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.RelativePressure senRelPreFilterOda(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {-230, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senTOda(redeclare package Medium = Medium1, T_start = T_start, allowFlowReversal = allowFlowReversal1, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {-180, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Actuators.Dampers.Exponential flapRet(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, final dpDamper_nominal = dpDamper_nominal) "Return air flap" annotation(
    Placement(visible = true, transformation(origin = {80, 60}, extent = {{200, 70}, {180, 90}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_a port_a3(redeclare package Medium = Medium2) "Fluid connector a3 (positive design flow direction is from port_a3 to port_b3)" annotation(
    Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -110}, {10, -90}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -110}, {10, -90}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b3(redeclare package Medium = Medium2) "Fluid connector b3 (positive design flow direction is from port_a3 to port_b3)" annotation(
    Placement(visible = true, transformation(origin = {20, 0}, extent = {{30, -110}, {50, -90}}, rotation = 0), iconTransformation(origin = {20, 0}, extent = {{30, -110}, {50, -90}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_a port_a4(redeclare package Medium = Medium2) "Fluid connector a4 (positive design flow direction is from port_a4 to port_b4)" annotation(
    Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -110}, {10, -90}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -110}, {10, -90}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b4(redeclare package Medium = Medium2) "Fluid connector b4 (positive design flow direction is from port_a4 to port_b4)" annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{30, -110}, {50, -90}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{30, -110}, {50, -90}}, rotation = 0)));
  AixLib.Fluid.Movers.FlowControlled_dp fanSup(redeclare package Medium = Medium1, energyDynamics = energyDynamics, T_start = T_start, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, redeclare AixLib.Fluid.Movers.Data.Generic per(hydraulicEfficiency = hydraulicEfficiency, motorEfficiency(eta = {0.95})), final inputType = AixLib.Fluid.Types.InputType.Continuous) "Supply air fan" annotation(
    Placement(visible = true, transformation(origin = {-96, -40}, extent = {{156, -10}, {176, 10}}, rotation = 0)));
  AixLib.Fluid.Movers.FlowControlled_dp fanRet(redeclare package Medium = Medium1, energyDynamics = energyDynamics, T_start = T_start, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, redeclare AixLib.Fluid.Movers.Data.Generic per(hydraulicEfficiency = hydraulicEfficiency, motorEfficiency(eta = {0.95})), final inputType = AixLib.Fluid.Types.InputType.Continuous) "Return air fan" annotation(
    Placement(visible = true, transformation(origin = {120, 140}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
  AixLib.Fluid.Humidifiers.GenericHumidifier_u humidifierSup(redeclare package Medium = Medium1, allowFlowReversal = allowFlowReversal1, dp_nominal = 10, energyDynamics = energyDynamics, m_flow_nominal = m1_flow_nominal) if useHumidifier "Steam or adiabatic humdifier in supply canal" annotation(
    Placement(visible = true, transformation(origin = {50, -40}, extent = {{130, -10}, {150, 10}}, rotation = 0)));
  AixLib.Fluid.Humidifiers.GenericHumidifier_u humidifierEha(redeclare package Medium = Medium1, dp_nominal = 10, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {70, 140}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senTRet(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, T_start = T_start) "Return air temperature sensor" annotation(
    Placement(visible = true, transformation(origin = {-240, 60}, extent = {{160, 70}, {140, 90}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senTExh(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, T_start = T_start) "Exhaust air temperature sensor" annotation(
    Placement(visible = true, transformation(origin = {-80, 60}, extent = {{-140, 70}, {-160, 90}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senTSup(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, final m_flow_nominal = m1_flow_nominal, T_start = T_start) "Supply air temperature sensor" annotation(
    Placement(visible = true, transformation(origin = {252, -40}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  AixLib.Fluid.Sensors.RelativeHumidityTwoPort senRelHumSup1(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, m_flow_nominal = m1_flow_nominal) "Supply air humidity sensor" annotation(
    Placement(visible = true, transformation(origin = {24, -40}, extent = {{180, -6}, {192, 6}}, rotation = 0)));
  AixLib.Fluid.Sensors.RelativeHumidityTwoPort senRelHumSup2(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, m_flow_nominal = m1_flow_nominal) "Supply air humidity sensor" annotation(
    Placement(visible = true, transformation(origin = {44, -40}, extent = {{180, -6}, {192, 6}}, rotation = 0)));
  AixLib.Fluid.Sensors.RelativeHumidityTwoPort senRelHumEha1(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, m_flow_nominal = m1_flow_nominal) "Supply air humidity sensor" annotation(
    Placement(visible = true, transformation(origin = {146, 140}, extent = {{-180, -6}, {-192, 6}}, rotation = 0)));
  AixLib.Fluid.Sensors.RelativeHumidityTwoPort senRelHumEha2(redeclare package Medium = Medium1, final allowFlowReversal = allowFlowReversal1, m_flow_nominal = m1_flow_nominal) "Supply air humidity sensor" annotation(
    Placement(visible = true, transformation(origin = {126, 140}, extent = {{-180, -6}, {-192, 6}}, rotation = 0)));
  parameter Modelica.Units.SI.PressureDifference dpDamper_nominal = 1 "Pressure drop of fully open dampers at nominal mass flow rate";
  AixLib.Fluid.HeatExchangers.DynamicHX cooler(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, Q_nom = 1000*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {-50, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.HeatExchangers.DynamicHX reHeater(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, Q_nom = 1000*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {-10, -34}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.HeatExchangers.DynamicHX heatExchangerKvsSup(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, Q_nom = 1000*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {-90, -34}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.HeatExchangers.DynamicHX heatExchangerKvsEha(redeclare package Medium1 = Medium1, redeclare package Medium2 = Medium2, Q_nom = 1000*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {-170, 134}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  mtpPackage.GenericAHUBus_extended genericAHUBus_extended annotation(
    Placement(visible = true, transformation(origin = {0, 180}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 154}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  AixLib.Fluid.Actuators.Dampers.Exponential flapEha(redeclare package Medium = Medium1, allowFlowReversal = allowFlowReversal1, dpDamper_nominal = dpDamper_nominal, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {-460, 60}, extent = {{200, 70}, {180, 90}}, rotation = 0)));
  AixLib.Fluid.Actuators.Dampers.Exponential flapSup(redeclare package Medium = Medium1, allowFlowReversal = allowFlowReversal1, dpDamper_nominal = dpDamper_nominal, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {460, -40}, extent = {{-190, -10}, {-170, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Temperature senTempSup1(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Temperature senTempSup2(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {-30, -70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Temperature senTempSup3(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {10, -70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Temperature senTempSup4(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {90, -70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.EnthalpyFlowRate senEntFloSup(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Pressure senPreSup(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {130, -70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Pressure senPreEha(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {-120, 160}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.FixedResistances.PressureDrop filterSup(redeclare package Medium = Medium1, dp_nominal = 10, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {160, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.RelativePressure senRelPreFilterSup(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {160, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.RelativePressure senRelPreFilterExa(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {160, 160}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  AixLib.Fluid.FixedResistances.PressureDrop filterExa(redeclare package Medium = Medium1, dp_nominal = 10, m_flow_nominal = m1_flow_nominal) annotation(
    Placement(visible = true, transformation(origin = {160, 140}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Pressure senPreOda(redeclare package Medium = Medium1) annotation(
    Placement(visible = true, transformation(origin = {-190, -70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.HeatExchangers.DynamicHX heatExchangerKvs(redeclare package Medium1 = Medium2, redeclare package Medium2 = Medium2, Q_nom = 4182*m1_flow_nominal, T1_start = T_start, T2_start = T_start, allowFlowReversal1 = allowFlowReversal1, allowFlowReversal2 = allowFlowReversal2, dT_nom = 1, dp1_nominal = 10, dp2_nominal = 10, energyDynamics = energyDynamics, m1_flow_nominal = m1_flow_nominal, m2_flow_nominal = m2_flow_nominal, massDynamics = massDynamics, tau1 = 2, tau2 = 8, tau_C = 10) annotation(
    Placement(visible = true, transformation(origin = {10, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  AixLib.Fluid.Sensors.Temperature senTempKvsIn(redeclare package Medium = Medium2) annotation(
    Placement(visible = true, transformation(origin = {-8, 40}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Temperature senTempKvsPreHeaterIn1(redeclare package Medium = Medium2) annotation(
    Placement(visible = true, transformation(origin = {-110, 40}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Temperature senTempKvsPreHeaterIn2(redeclare package Medium = Medium2) annotation(
    Placement(visible = true, transformation(origin = {-130, -2}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  AixLib.Fluid.Sensors.Temperature senTempKvsPreHeaterOut(redeclare package Medium = Medium2) annotation(
    Placement(visible = true, transformation(origin = {-170, 20}, extent = {{10, 10}, {-10, -10}}, rotation = -90)));
  AixLib.Fluid.Sensors.Pressure senPreKvsPreHeaterOut(redeclare package Medium = Medium2) annotation(
    Placement(visible = true, transformation(origin = {-190, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  AixLib.Fluid.Sensors.Temperature senTempKvsOut(redeclare package Medium = Medium2) annotation(
    Placement(visible = true, transformation(origin = {-8, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.Temperature senTempKvsHeatExchangerEhaIn(redeclare package Medium = Medium2) annotation(
    Placement(visible = true, transformation(origin = {-230, 108}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  AixLib.Fluid.Sensors.Temperature senTempKvsHeatExchangerEhaOut(redeclare package Medium = Medium2) annotation(
    Placement(visible = true, transformation(origin = {-120, 120}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  AixLib.Fluid.Sensors.EnthalpyFlowRate senEntKvsHeatExchangerEha(redeclare package Medium = Medium2) annotation(
    Placement(visible = true, transformation(origin = {-190, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  AixLib.Fluid.Movers.SpeedControlled_y pump(redeclare package Medium = Medium2, redeclare AixLib.Fluid.Movers.Data.Pumps.Wilo.Stratos25slash1to4 per(speeds_rpm = 1800*{0, 0.5, 1}, constantSpeed_rpm = 1800), use_inputFilter = false, inputType = AixLib.Fluid.Types.InputType.Constant, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState) annotation(
    Placement(visible = true, transformation(origin = {-30, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.MixingVolumes.MixingVolume junc1(redeclare package Medium = Medium2, V = vol, nPorts = 3) annotation(
    Placement(visible = true, transformation(origin = {-70, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.MixingVolumes.MixingVolume junc2(redeclare package Medium = Medium2, V = vol, nPorts = 4) annotation(
    Placement(visible = true, transformation(origin = {-90, 100}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.Actuators.Valves.ThreeWayTable valveKvs1(redeclare package Medium = Medium2, dpValve_nominal = 6000, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, flowCharacteristics1 = AixLib.Fluid.Actuators.Valves.Data.Linear(), flowCharacteristics3(phi = {0.001, 0.3, 1}, y = {0, 0.5, 1}), m_flow_nominal = 2, use_inputFilter = false) "Valve model, linear opening characteristics" annotation(
    Placement(visible = true, transformation(origin = {-160, 80}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  AixLib.Fluid.Actuators.Valves.ThreeWayTable valveKvs2(redeclare package Medium = Medium2, dpValve_nominal = 6000, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, flowCharacteristics1 = AixLib.Fluid.Actuators.Valves.Data.Linear(), flowCharacteristics3(phi = {0.001, 0.3, 1}, y = {0, 0.5, 1}), m_flow_nominal = 2, use_inputFilter = false) "Valve model, linear opening characteristics" annotation(
    Placement(visible = true, transformation(origin = {-130, 80}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.Actuators.Valves.TwoWayTable valveKvs3(redeclare package Medium = Medium2, use_inputFilter = false, from_dp = true, flowCharacteristics = datVal, CvData = AixLib.Fluid.Types.CvTypes.Kv, Kv = 0.65, m_flow_nominal = 0.04) "Valve model with opening characteristics based on a table" annotation(
    Placement(visible = true, transformation(origin = {-70, 20}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
  Modelica.Fluid.Sources.Boundary_pT boundary_a1(redeclare package Medium = Medium1, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-370, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Sources.Boundary_pT boundary_b1(redeclare package Medium = Medium1, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {370, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Fluid.Sources.Boundary_pT boundary_a2(redeclare package Medium = Medium1, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {372, 140}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Fluid.Sources.Boundary_pT boundary_b2(redeclare package Medium = Medium1, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-370, 140}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Sources.Boundary_pT boundary_a3(redeclare package Medium = Medium2, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {40, -130}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Fluid.Sources.Boundary_pT boundary_b3(redeclare package Medium = Medium2, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {60, -170}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Fluid.Sources.Boundary_pT boundary_a4(redeclare package Medium = Medium2, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-40, -132}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Fluid.Sources.Boundary_pT boundary_b4(redeclare package Medium = Medium2, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-60, -172}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Fluid.Sources.Boundary_pT boundary_kvs(redeclare package Medium = Medium2, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-48, 110}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant humidiferSet(k = 0.5) annotation(
    Placement(visible = true, transformation(origin = {-30, 270}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant setFlaps(k = 1) annotation(
    Placement(visible = true, transformation(origin = {-110, 230}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant dpFansSet(k = 100) annotation(
    Placement(visible = true, transformation(origin = {-70, 250}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant setValves(k = 0) annotation(
    Placement(visible = true, transformation(origin = {30, 230}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant pumpSet(k = 1) annotation(
    Placement(visible = true, transformation(origin = {-70, 210}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  Modelica.Blocks.Continuous.FirstOrder PT1_airIn(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(visible = true, transformation(origin = {252, -20}, extent = {{4, -4}, {-4, 4}}, rotation = 270)));
  Modelica.Blocks.Continuous.FirstOrder PT1_airIn1(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) "Oda sensor delay" annotation(
    Placement(visible = true, transformation(origin = {-180, -14}, extent = {{4, -4}, {-4, 4}}, rotation = 270)));
  Modelica.Blocks.Continuous.FirstOrder PT1_airIn2(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(visible = true, transformation(origin = {-230, 166}, extent = {{4, -4}, {-4, 4}}, rotation = 270)));
  Modelica.Blocks.Continuous.FirstOrder PT1_airIn3(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(visible = true, transformation(origin = {-90, 162}, extent = {{4, -4}, {-4, 4}}, rotation = 270)));
equation
  connect(senTSup.T, PT1_airIn.u) annotation(
    Line(points = {{252, -31.2}, {252, -25.2}}, color = {0, 0, 127}));
  connect(senTExh.T, PT1_airIn2.u) annotation(
    Line(points = {{-230, 151}, {-230, 161}}, color = {0, 0, 127}));
  connect(senTRet.T, PT1_airIn3.u) annotation(
    Line(points = {{-90, 151}, {-90, 157.2}}, color = {0, 0, 127}));
  connect(heatExchangerKvsSup.port_b1, cooler.port_a1) annotation(
    Line(points = {{-80, -40}, {-60, -40}}, color = {0, 127, 255}));
  connect(cooler.port_b2, port_b4) annotation(
    Line(points = {{-60, -52}, {-60, -100}}, color = {0, 127, 255}));
  connect(cooler.port_a2, port_a4) annotation(
    Line(points = {{-40, -52}, {-40, -100}}, color = {0, 127, 255}));
  connect(flapOda.y, genericAHUBus_extended.flapOdaSet) annotation(
    Line(points = {{-270, -28}, {-270, 40}, {-336, 40}, {-336, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(flapOda.y_actual, genericAHUBus_extended.flapOdaMea) annotation(
    Line(points = {{-265, -33}, {-265, 46}, {-330, 46}, {-330, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(flapEha.y_actual, genericAHUBus_extended.flapEhaMea) annotation(
    Line(points = {{-275, 147}, {-275, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(flapEha.y, genericAHUBus_extended.flapEhaSet) annotation(
    Line(points = {{-270, 152}, {-270, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(PT1_airIn2.y, genericAHUBus_extended.TEhaMea) annotation(
    Line(points = {{-230, 170.4}, {-230, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(fanRet.dp_in, genericAHUBus_extended.dpFanEtaSet) annotation(
    Line(points = {{120, 152}, {120, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(fanRet.dp_actual, genericAHUBus_extended.dpFanEtaMea) annotation(
    Line(points = {{109, 145}, {100, 145}, {100, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(flapRet.y, genericAHUBus_extended.flapEtaSet) annotation(
    Line(points = {{270, 152}, {270, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(reHeater.port_b1, fanSup.port_a) annotation(
    Line(points = {{0, -40}, {60, -40}}, color = {0, 127, 255}));
  connect(senTSup.port_b, flapSup.port_a) annotation(
    Line(points = {{260, -40}, {270, -40}}, color = {0, 127, 255}));
  connect(flapSup.port_b, port_b1) annotation(
    Line(points = {{290, -40}, {300, -40}}, color = {0, 127, 255}));
  connect(senTOda.T, PT1_airIn1.u) annotation(
    Line(points = {{-180, -29}, {-180, -19}}, color = {0, 0, 127}));
  connect(port_a2, flapRet.port_a) annotation(
    Line(points = {{300, 140}, {280, 140}}));
  connect(senTExh.port_b, flapEha.port_a) annotation(
    Line(points = {{-240, 140}, {-260, 140}}, color = {0, 127, 255}));
  connect(flapEha.port_b, port_b2) annotation(
    Line(points = {{-280, 140}, {-300, 140}}, color = {0, 127, 255}));
  connect(humidifierSup.u, genericAHUBus_extended.humSetSup) annotation(
    Line(points = {{179, -34}, {176, -34}, {176, 54}, {342, 54}, {342, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(filterOda.port_b, senTOda.port_a) annotation(
    Line(points = {{-220, -40}, {-190, -40}}, color = {0, 127, 255}));
  connect(flapOda.port_b, filterOda.port_a) annotation(
    Line(points = {{-260, -40}, {-240, -40}}, color = {0, 127, 255}));
  connect(flapOda.port_a, port_a1) annotation(
    Line(points = {{-280, -40}, {-300, -40}}, color = {0, 127, 255}));
  connect(senRelPreFilterOda.port_a, filterOda.port_a) annotation(
    Line(points = {{-240, -70}, {-250, -70}, {-250, -40}, {-240, -40}}, color = {0, 127, 255}));
  connect(filterOda.port_b, senRelPreFilterOda.port_b) annotation(
    Line(points = {{-220, -40}, {-210, -40}, {-210, -70}, {-220, -70}}, color = {0, 127, 255}));
  connect(cooler.port_b1, reHeater.port_a1) annotation(
    Line(points = {{-40, -40}, {-20, -40}}, color = {0, 127, 255}));
  connect(heatExchangerKvsSup.port_b1, senTempSup1.port) annotation(
    Line(points = {{-80, -40}, {-70, -40}, {-70, -60}}, color = {0, 127, 255}));
  connect(senTempSup2.port, cooler.port_b1) annotation(
    Line(points = {{-30, -60}, {-30, -40}, {-40, -40}}, color = {0, 127, 255}));
  connect(reHeater.port_b1, senTempSup3.port) annotation(
    Line(points = {{0, -40}, {10, -40}, {10, -60}}, color = {0, 127, 255}));
  connect(fanSup.port_b, senTempSup4.port) annotation(
    Line(points = {{80, -40}, {90, -40}, {90, -60}}, color = {0, 127, 255}));
  connect(fanSup.port_b, senEntFloSup.port_a) annotation(
    Line(points = {{80, -40}, {100, -40}}, color = {0, 127, 255}));
  connect(senPreSup.port, senEntFloSup.port_b) annotation(
    Line(points = {{130, -60}, {130, -40}, {120, -40}}, color = {0, 127, 255}));
  connect(senEntFloSup.port_b, filterSup.port_a) annotation(
    Line(points = {{120, -40}, {150, -40}}, color = {0, 127, 255}));
  connect(senRelPreFilterSup.port_a, filterSup.port_a) annotation(
    Line(points = {{150, -70}, {146, -70}, {146, -40}, {150, -40}}, color = {0, 127, 255}));
  connect(senRelPreFilterSup.port_b, filterSup.port_b) annotation(
    Line(points = {{170, -70}, {176, -70}, {176, -40}, {170, -40}}, color = {0, 127, 255}));
  connect(filterSup.port_b, humidifierSup.port_a) annotation(
    Line(points = {{170, -40}, {180, -40}}, color = {0, 127, 255}));
  connect(senRelPreFilterExa.port_a, filterExa.port_a) annotation(
    Line(points = {{170, 160}, {180, 160}, {180, 140}, {170, 140}}, color = {0, 127, 255}));
  connect(filterExa.port_b, senRelPreFilterExa.port_b) annotation(
    Line(points = {{150, 140}, {140, 140}, {140, 160}, {150, 160}}, color = {0, 127, 255}));
  connect(fanRet.port_a, filterExa.port_b) annotation(
    Line(points = {{130, 140}, {150, 140}}, color = {0, 127, 255}));
  connect(filterExa.port_a, flapRet.port_b) annotation(
    Line(points = {{170, 140}, {260, 140}}, color = {0, 127, 255}));
  connect(fanRet.P, genericAHUBus_extended.powerFanRetMea) annotation(
    Line(points = {{109, 149}, {103, 149}, {103, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(flapRet.y_actual, genericAHUBus_extended.flapEtaMea) annotation(
    Line(points = {{265, 147}, {210, 147}, {210, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(fanSup.dp_in, genericAHUBus_extended.dpFanSupSet) annotation(
    Line(points = {{70, -28}, {70, 60}, {336, 60}, {336, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(fanSup.P, genericAHUBus_extended.powerFanSupMea) annotation(
    Line(points = {{81, -31}, {84, -31}, {84, 58}, {338, 58}, {338, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(fanSup.dp_actual, genericAHUBus_extended.dpFanSupMea) annotation(
    Line(points = {{81, -35}, {86, -35}, {86, 56}, {340, 56}, {340, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(PT1_airIn.y, genericAHUBus_extended.TSupMea) annotation(
    Line(points = {{252, -16}, {252, 48}, {348, 48}, {348, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(flapSup.y, genericAHUBus_extended.flapSupSet) annotation(
    Line(points = {{280, -28}, {280, 46}, {350, 46}, {350, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(flapSup.y_actual, genericAHUBus_extended.flapSupMea) annotation(
    Line(points = {{285, -33}, {288, -33}, {288, 44}, {352, 44}, {352, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(senPreOda.port, filterOda.port_b) annotation(
    Line(points = {{-190, -60}, {-200, -60}, {-200, -40}, {-220, -40}}, color = {0, 127, 255}));
  connect(senTOda.port_b, heatExchangerKvsSup.port_a1) annotation(
    Line(points = {{-170, -40}, {-100, -40}}, color = {0, 127, 255}));
  connect(senTempKvsIn.port, heatExchangerKvs.port_b2) annotation(
    Line(points = {{-8, 50}, {4, 50}, {4, 60}}, color = {0, 127, 255}));
  connect(senTempKvsOut.port, heatExchangerKvs.port_a2) annotation(
    Line(points = {{-8, 90}, {4, 90}, {4, 80}}, color = {0, 127, 255}));
  connect(senTExh.port_a, heatExchangerKvsEha.port_b1) annotation(
    Line(points = {{-220, 140}, {-180, 140}}, color = {0, 127, 255}));
  connect(senTRet.port_b, senPreEha.port) annotation(
    Line(points = {{-100, 140}, {-120, 140}, {-120, 150}}, color = {0, 127, 255}));
  connect(senTRet.port_b, heatExchangerKvsEha.port_a1) annotation(
    Line(points = {{-100, 140}, {-160, 140}}, color = {0, 127, 255}));
  connect(junc1.ports[3], senTempKvsPreHeaterIn1.port) annotation(
    Line(points = {{-70, 50}, {-110, 50}}, color = {0, 127, 255}));
  connect(senTempKvsPreHeaterIn1.port, senTempKvsPreHeaterIn2.port) annotation(
    Line(points = {{-110, 50}, {-140, 50}, {-140, -2}}, color = {0, 127, 255}));
  connect(senTempKvsPreHeaterIn2.port, heatExchangerKvsSup.port_a2) annotation(
    Line(points = {{-140, -2}, {-140, -20}, {-30, -20}, {-30, -28}, {-80, -28}}, color = {0, 127, 255}));
  connect(PT1_airIn1.y, genericAHUBus_extended.TOdaMea) annotation(
    Line(points = {{-180, -10}, {-180, 0}, {-260, 0}, {-260, 52}, {-324, 52}, {-324, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(heatExchangerKvsSup.port_b2, senTempKvsPreHeaterOut.port) annotation(
    Line(points = {{-100, -28}, {-160, -28}, {-160, 20}}, color = {0, 127, 255}));
  connect(senPreKvsPreHeaterOut.port, senTempKvsPreHeaterOut.port) annotation(
    Line(points = {{-180, 40}, {-160, 40}, {-160, 20}}, color = {0, 127, 255}));
  connect(valveKvs1.port_2, senTempKvsPreHeaterOut.port) annotation(
    Line(points = {{-160, 70}, {-160, 20}}, color = {0, 127, 255}));
  connect(junc1.ports[3], valveKvs3.port_a) annotation(
    Line(points = {{-70, 50}, {-70, 30}}, color = {0, 127, 255}));
  connect(valveKvs3.port_b, reHeater.port_a2) annotation(
    Line(points = {{-70, 10}, {-70, 0}, {12, 0}, {12, -28}, {0, -28}}, color = {0, 127, 255}));
  connect(reHeater.port_b2, junc2.ports[1]) annotation(
    Line(points = {{-20, -28}, {-20, -10}, {-90, -10}, {-90, 90}}, color = {0, 127, 255}));
  connect(heatExchangerKvsEha.port_b2, valveKvs2.port_3) annotation(
    Line(points = {{-160, 128}, {-130, 128}, {-130, 90}}, color = {0, 127, 255}));
  connect(heatExchangerKvsEha.port_b2, senTempKvsHeatExchangerEhaOut.port) annotation(
    Line(points = {{-160, 128}, {-130, 128}, {-130, 120}}, color = {0, 127, 255}));
  connect(valveKvs2.port_2, junc2.ports[2]) annotation(
    Line(points = {{-120, 80}, {-90, 80}, {-90, 90}}, color = {0, 127, 255}));
  connect(junc2.ports[3], senTempKvsOut.port) annotation(
    Line(points = {{-90, 90}, {-8, 90}}, color = {0, 127, 255}));
  connect(valveKvs1.port_3, valveKvs2.port_1) annotation(
    Line(points = {{-150, 80}, {-140, 80}}, color = {0, 127, 255}));
  connect(senEntKvsHeatExchangerEha.port_a, valveKvs1.port_1) annotation(
    Line(points = {{-190, 100}, {-190, 94}, {-160, 94}, {-160, 90}}, color = {0, 127, 255}));
  connect(senTempKvsHeatExchangerEhaIn.port, senEntKvsHeatExchangerEha.port_b) annotation(
    Line(points = {{-220, 108}, {-210, 108}, {-210, 128}, {-190, 128}, {-190, 120}}, color = {0, 127, 255}));
  connect(senEntKvsHeatExchangerEha.port_b, heatExchangerKvsEha.port_a2) annotation(
    Line(points = {{-190, 120}, {-190, 128}, {-180, 128}}, color = {0, 127, 255}));
  connect(valveKvs3.y, genericAHUBus_extended.y3) annotation(
    Line(points = {{-58, 20}, {30, 20}, {30, 130}, {0, 130}, {0, 180}}, color = {0, 0, 127}));
  connect(valveKvs2.y, genericAHUBus_extended.y2) annotation(
    Line(points = {{-130, 68}, {-130, 60}, {-144, 60}, {-144, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(valveKvs1.y, genericAHUBus_extended.y1) annotation(
    Line(points = {{-172, 80}, {-180, 80}, {-180, 60}, {-144, 60}, {-144, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(PT1_airIn3.y, genericAHUBus_extended.TEtaMea) annotation(
    Line(points = {{-90, 166}, {-90, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(humidifierSup.port_b, senRelHumSup1.port_a) annotation(
    Line(points = {{200, -40}, {204, -40}}, color = {0, 127, 255}));
  connect(senRelHumSup1.port_b, senRelHumSup2.port_a) annotation(
    Line(points = {{216, -40}, {224, -40}}, color = {0, 127, 255}));
  connect(senRelHumSup2.port_b, senTSup.port_a) annotation(
    Line(points = {{236, -40}, {244, -40}}, color = {0, 127, 255}));
  connect(senRelHumSup1.phi, genericAHUBus_extended.relHumSupMea) annotation(
    Line(points = {{210, -33}, {210, 50}, {346, 50}, {346, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(senRelHumEha1.port_b, senRelHumEha2.port_a) annotation(
    Line(points = {{-46, 140}, {-54, 140}}, color = {0, 127, 255}));
  connect(senRelHumEha2.port_b, senTRet.port_a) annotation(
    Line(points = {{-66, 140}, {-80, 140}}, color = {0, 127, 255}));
  connect(senRelHumEha1.phi, genericAHUBus_extended.relHumEtaMea) annotation(
    Line(points = {{-40, 147}, {-40, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(humidifierSup.powerEva, genericAHUBus_extended.powerHumSupMea) annotation(
    Line(points = {{201, -30}, {208, -30}, {208, 52}, {344, 52}, {344, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(heatExchangerKvs.port_a1, port_a3) annotation(
    Line(points = {{16, 60}, {40, 60}, {40, -100}}, color = {0, 127, 255}));
  connect(heatExchangerKvs.port_b1, port_b3) annotation(
    Line(points = {{16, 80}, {50, 80}, {50, -70}, {60, -70}, {60, -100}}, color = {0, 127, 255}));
  connect(boundary_b2.ports[1], port_b2) annotation(
    Line(points = {{-360, 140}, {-300, 140}}, color = {0, 127, 255}));
  connect(boundary_a1.ports[1], port_a1) annotation(
    Line(points = {{-360, -40}, {-300, -40}}, color = {0, 127, 255}));
  connect(boundary_b1.ports[1], port_b1) annotation(
    Line(points = {{360, -40}, {300, -40}}, color = {0, 127, 255}));
  connect(boundary_a2.ports[1], port_a2) annotation(
    Line(points = {{362, 140}, {300, 140}}, color = {0, 127, 255}));
  connect(boundary_b3.ports[1], port_b3) annotation(
    Line(points = {{60, -160}, {60, -100}}, color = {0, 127, 255}));
  connect(boundary_a3.ports[1], port_a3) annotation(
    Line(points = {{40, -120}, {40, -100}}, color = {0, 127, 255}));
  connect(boundary_a4.ports[1], port_a4) annotation(
    Line(points = {{-40, -122}, {-40, -100}}, color = {0, 127, 255}));
  connect(boundary_b4.ports[1], port_b4) annotation(
    Line(points = {{-60, -162}, {-60, -100}}, color = {0, 127, 255}));
  connect(humidiferSet.y, genericAHUBus_extended.humSetSup) annotation(
    Line(points = {{-18, 270}, {0, 270}, {0, 180}}, color = {0, 0, 127}));
  connect(humidiferSet.y, genericAHUBus_extended.humSetEta) annotation(
    Line(points = {{-18, 270}, {0, 270}, {0, 180}}, color = {0, 0, 127}));
  connect(dpFansSet.y, genericAHUBus_extended.dpFanEtaSet) annotation(
    Line(points = {{-58, 250}, {0, 250}, {0, 180}}, color = {0, 0, 127}));
  connect(setFlaps.y, genericAHUBus_extended.flapEhaSet) annotation(
    Line(points = {{-98, 230}, {0, 230}, {0, 180}}, color = {0, 0, 127}));
  connect(setFlaps.y, genericAHUBus_extended.flapEtaSet) annotation(
    Line(points = {{-98, 230}, {0, 230}, {0, 180}}, color = {0, 0, 127}));
  connect(setFlaps.y, genericAHUBus_extended.flapOdaSet) annotation(
    Line(points = {{-98, 230}, {0, 230}, {0, 180}}, color = {0, 0, 127}));
  connect(setFlaps.y, genericAHUBus_extended.flapSupSet) annotation(
    Line(points = {{-98, 230}, {0, 230}, {0, 180}}, color = {0, 0, 127}));
  connect(setValves.y, genericAHUBus_extended.y1) annotation(
    Line(points = {{20, 230}, {0, 230}, {0, 180}}, color = {0, 0, 127}));
  connect(setValves.y, genericAHUBus_extended.y2) annotation(
    Line(points = {{20, 230}, {0, 230}, {0, 180}}, color = {0, 0, 127}));
  connect(setValves.y, genericAHUBus_extended.y3) annotation(
    Line(points = {{20, 230}, {0, 230}, {0, 180}}, color = {0, 0, 127}));
  connect(junc1.ports[3], pump.port_b) annotation(
    Line(points = {{-70, 50}, {-40, 50}}, color = {0, 127, 255}));
  connect(pump.port_a, senTempKvsIn.port) annotation(
    Line(points = {{-20, 50}, {-8, 50}}, color = {0, 127, 255}));
  connect(junc2.ports[4], boundary_kvs.ports[1]) annotation(
    Line(points = {{-90, 90}, {-48, 90}, {-48, 100}}, color = {0, 127, 255}));
  connect(pump.y, pumpSet.y) annotation(
    Line(points = {{-30, 62}, {-30, 210}, {-58, 210}}, color = {0, 0, 127}));
  connect(dpFansSet.y, genericAHUBus_extended.dpFanSupSet) annotation(
    Line(points = {{-58, 250}, {0, 250}, {0, 180}}, color = {0, 0, 127}));
  connect(humidifierEha.port_a, fanRet.port_b) annotation(
    Line(points = {{80, 140}, {110, 140}}, color = {0, 127, 255}));
  connect(humidifierEha.port_b, senRelHumEha1.port_a) annotation(
    Line(points = {{60, 140}, {-34, 140}}, color = {0, 127, 255}));
  connect(humidifierEha.powerEva, genericAHUBus_extended.powerHumEtaMea) annotation(
    Line(points = {{60, 150}, {48, 150}, {48, 180}, {0, 180}}, color = {0, 0, 127}));
  connect(humidifierEha.u, genericAHUBus_extended.humSetEta) annotation(
    Line(points = {{82, 146}, {90, 146}, {90, 180}, {0, 180}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-300, -100}, {300, 200}}), graphics = {Rectangle(origin = {0, 10}, fillColor = {170, 255, 127}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-200, 90}, {200, -90}}), Text(origin = {0, 10}, extent = {{-200, 90}, {200, -90}}, textString = "RLT TYP B2", fontSize = 72)}),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-300, -100}, {300, 200}})),
    Documentation(revisions = "<html>
<ul>
<li>October 29, 2019, by Alexander K&uuml;mpel:<br/>First implementation</li>
</ul>
</html>", info = "<html>
<p>The GenericAHU is an air-handling unit model with detailed hydraulic system of the preheater, heater and cooler. The ahu includes a heat exchanger for heat recovery and a humidifier for the supply air. The humidifier can be insert steam or water that evaporates completely (adiabatic). Further, the ahu includes an adiabatic humidifier in the return air chanal in order to cool the return air and use the heat recovery heat excahnger to cool the supply air. The preheater, steam humdifier and adiabatic humidifier are conditional and can be deactivated.</p>
</html>"));
end RLTTypB2_withBoundary;
