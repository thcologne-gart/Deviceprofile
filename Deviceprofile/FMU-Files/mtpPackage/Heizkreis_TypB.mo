within mtpPackage;

model Heizkreis_TypB "Modell für Heizkreis Typ B-Beimischschaltung"
  package Medium = AixLib.Media.Water "Medium in the components";
  
  // Assumptions
  parameter Boolean allowFlowReversal = true "= true to allow flow reversal, false restricts to design direction (port_a -> port_b)" annotation(
    Dialog(tab = "Assumptions"),
    Evaluate = true);
  parameter Boolean allowFlowReversal1 = true "= false to simplify equations, assuming, but not enforcing, no flow reversal for medium 1" annotation(
    Dialog(tab = "Assumptions"),
    Evaluate = true);
  parameter Boolean allowFlowReversal2 = true "= false to simplify equations, assuming, but not enforcing, no flow reversal for medium 2" annotation(
    Dialog(tab = "Assumptions"),
    Evaluate = true);
  // General Parameters
  parameter String pipeModel = "SimplePipe" annotation(
    choices(choice = "SimplePipe", choice = "PlugFlowPipe"),
    Dialog(group = "Parameters"));
  parameter Modelica.Units.SI.Length length "Pipe length of all pipes (can be overwritten in each pipe)" annotation(
    Dialog(group = "Pipes"));
  parameter AixLib.DataBase.Pipes.PipeBaseDataDefinition parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_6x1() "Pipe type and diameter (can be overwritten in each pipe)" annotation(
    choicesAllMatching = true,
    Dialog(group = "Pipes"));
  parameter AixLib.DataBase.Pipes.InsulationBaseDataDefinition parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc() "Insulation Type (can be overwritten in each pipe)" annotation(
    choicesAllMatching = true,
    Dialog(group = "Pipes"));
  parameter Real Kv "Kv value of valve (can be overwritten in the valve)" annotation(
    Dialog(group = "Actuators"));
  parameter Modelica.Units.SI.MassFlowRate m_flow_nominal(min = 0) "Nominal mass flow rate" annotation(
    Dialog(group = "Nominal condition"));
  // Initialization
  parameter Modelica.Units.SI.Temperature T_start = 303.15 "Initialization temperature" annotation(
    Dialog(tab = "Initialization"));
  // Advanced
  parameter Modelica.Units.SI.Time tau = 15 "Time Constant for PT1 behavior of temperature sensors" annotation(
    Dialog(tab = "Advanced"));
  parameter Modelica.Units.SI.Temperature T_amb = 298.15 "Ambient temperature for heat loss" annotation(
    Dialog(tab = "Advanced"));
  parameter Modelica.Units.SI.Time tauHeaTra = parameterPipe.d_i*parameterPipe.d_i/4*1000*4180*(log(parameterPipe.d_i/parameterPipe.d_o)/2/parameterPipe.lambda + log(parameterPipe.d_o/parameterPipe.d_o*(1 + parameterIso.factor))/2/parameterIso.lambda + 1/hCon/parameterPipe.d_o*(1 + parameterIso.factor)) "Time constant for heat transfer of temperature sensors to ambient" annotation(
    Dialog(tab = "Advanced"));
  parameter Modelica.Units.SI.CoefficientOfHeatTransfer hCon = 4 "Convection heat transfer coeffient for all pipes" annotation(
    Dialog(tab = "Advanced"));
  // Dynamics
  parameter Modelica.Fluid.Types.Dynamics energyDynamics = Modelica.Fluid.Types.Dynamics.DynamicFreeInitial "Type of energy balance: dynamic (3 initialization options) or steady state" annotation(
    Evaluate = true,
    Dialog(tab = "Dynamics", group = "Equations"));
  parameter Modelica.Fluid.Types.Dynamics massDynamics = Modelica.Fluid.Types.Dynamics.DynamicFreeInitial "Type of mass balance: dynamic (3 initialization options) or steady state" annotation(
    Evaluate = true,
    Dialog(tab = "Dynamics", group = "Equations"));
  //-----parameters-----
  parameter Modelica.Units.SI.Volume vol = 0.0005 "Mixing Volume" annotation(
    Dialog(tab = "Advanced"));
  //-----ports-----
  Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare final package Medium = Medium, m_flow(min = if allowFlowReversal1 then -Modelica.Constants.inf else 0), h_outflow(start = Medium.h_default, nominal = Medium.h_default)) "Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)" annotation(
    Placement(visible = true, transformation(origin = {-198, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-200, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(redeclare final package Medium = Medium, m_flow(max = if allowFlowReversal1 then +Modelica.Constants.inf else 0), h_outflow(start = Medium.h_default, nominal = Medium.h_default)) "Fluid connector b1 (positive design flow direction is from port_a1 to port_b1)" annotation(
    Placement(visible = true, transformation(origin = {200, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {200, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_a port_a2(redeclare final package Medium = Medium, m_flow(min = if allowFlowReversal2 then -Modelica.Constants.inf else 0), h_outflow(start = Medium.h_default, nominal = Medium.h_default)) "Fluid connector a2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {200, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {200, -98}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2(redeclare final package Medium = Medium, m_flow(max = if allowFlowReversal2 then +Modelica.Constants.inf else 0), h_outflow(start = Medium.h_default, nominal = Medium.h_default)) "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {-200, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-200, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //-----pipes-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe1(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {-70, 80}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.FixedResistances.GenericPipe pipe2(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {60, 80}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  AixLib.Fluid.FixedResistances.GenericPipe pipe3(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {110, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.FixedResistances.GenericPipe pipe4(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {-70, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.FixedResistances.GenericPipe pipe5(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {-130, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  //-----pump-----
  AixLib.Fluid.Movers.SpeedControlled_y pump(redeclare package Medium = Medium, redeclare AixLib.Fluid.Movers.Data.Pumps.Wilo.Stratos25slash1to4 per(speeds_rpm = 1800*{0, 0.5, 1}, constantSpeed_rpm = 1800), use_inputFilter = false, inputType = AixLib.Fluid.Types.InputType.Constant, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState) annotation(
    Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant y(k = 1) annotation(
    Placement(visible = true, transformation(origin = {60, 80}, extent = {{-100, 40}, {-80, 60}}, rotation = 0)));
  //-----valve-----
  AixLib.Fluid.Actuators.Valves.ThreeWayTable valve(redeclare package Medium = Medium, dpValve_nominal = 6000, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, flowCharacteristics1 = AixLib.Fluid.Actuators.Valves.Data.Linear(), flowCharacteristics3(phi = {0.001, 0.3, 1}, y = {0, 0.5, 1}), m_flow_nominal = 2, use_inputFilter = false) "Valve model, linear opening characteristics" annotation(
    Placement(visible = true, transformation(origin = {-130, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.MixingVolumes.MixingVolume junc45(redeclare package Medium = Medium, V = vol, nPorts = 3) annotation(
    Placement(visible = true, transformation(origin = {-130, -90}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  //-----sensors-----
  AixLib.Fluid.Sensors.TemperatureTwoPort senT_b1(redeclare package Medium = Medium, TAmb = T_amb, T_start = T_start, allowFlowReversal = allowFlowReversal, m_flow_nominal = 0.3, tau = 0.01, tauHeaTra = tauHeaTra, transferHeat = true) annotation(
    Placement(visible = true, transformation(origin = {150, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senT_a2(redeclare package Medium = Medium, TAmb = T_amb, T_start = T_start, allowFlowReversal = allowFlowReversal, m_flow_nominal = 0.3, tau = 0.01, tauHeaTra = tauHeaTra, transferHeat = true) annotation(
    Placement(visible = true, transformation(origin = {150, -80}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senT_WMZ2(redeclare package Medium = Medium, TAmb = T_amb, T_start = T_start, allowFlowReversal = allowFlowReversal, m_flow_nominal = 0.3, tau = 0.01, tauHeaTra = tauHeaTra, transferHeat = true) annotation(
    Placement(visible = true, transformation(origin = {0, -80}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senT_WMZ1(redeclare package Medium = Medium, TAmb = T_amb, T_start = T_start, allowFlowReversal = allowFlowReversal, m_flow_nominal = 0.3, tau = 0.01, tauHeaTra = tauHeaTra, transferHeat = true) annotation(
    Placement(visible = true, transformation(origin = {110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder PT1_WMZ1(final T = tau, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start) annotation(
    Placement(visible = true, transformation(origin = {90, 130}, extent = {{10, -10}, {-10, 10}}, rotation = 270)));
Modelica.Blocks.Continuous.FirstOrder PT1_b1(final T = tau, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start) annotation(
    Placement(visible = true, transformation(origin = {150, 130}, extent = {{10, -10}, {-10, 10}}, rotation = 270)));
  Modelica.Blocks.Continuous.FirstOrder PT1_WMZ2(final T = tau, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start) annotation(
    Placement(visible = true, transformation(origin = {0, -130}, extent = {{10, 10}, {-10, -10}}, rotation = -270)));
  Modelica.Blocks.Continuous.FirstOrder PT1_a2(final T = tau, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start) annotation(
    Placement(visible = true, transformation(origin = {150, -130}, extent = {{10, 10}, {-10, -10}}, rotation = -270)));  
  
  AixLib.Fluid.Sensors.VolumeFlowRate senVolFlo(redeclare package Medium = Medium, m_flow_nominal = 10, tau = 1) annotation(
    Placement(visible = true, transformation(origin = {60, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----boundarys-----
  Modelica.Blocks.Sources.Ramp ramp(duration = 1, height = 1, offset = 0) annotation(
    Placement(visible = true, transformation(origin = {-140, 70}, extent = {{-40, 50}, {-20, 70}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = T_amb) annotation(
    Placement(visible = true, transformation(origin = {108, 20}, extent = {{72, -28}, {56, -12}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature annotation(
    Placement(visible = true, transformation(origin = {128, 0}, extent = {{-8, -8}, {8, 8}}, rotation = 180)));
  //-----consumer-----
  AixLib.Systems.HydraulicModules.SimpleConsumer consumer(redeclare package Medium = Medium, Q_flow_fixed = 1000, functionality = "Q_flow_fixed") annotation(
    Placement(visible = true, transformation(origin = {252, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
equation
  connect(y.y, pump.y) annotation(
    Line(points = {{-18, 130}, {0, 130}, {0, 92}}, color = {0, 0, 127}));
  connect(pipe1.port_b, pump.port_a) annotation(
    Line(points = {{-60, 80}, {-10, 80}}, color = {0, 127, 255}));
  connect(pump.port_b, pipe2.port_a) annotation(
    Line(points = {{10, 80}, {50, 80}}, color = {0, 127, 255}));
  connect(senT_b1.port_b, port_b1) annotation(
    Line(points = {{160, 80}, {200, 80}}, color = {0, 127, 255}));
  connect(senT_a2.port_a, port_a2) annotation(
    Line(points = {{160, -80}, {200, -80}}, color = {0, 127, 255}));
  connect(senT_a2.port_b, pipe3.port_a) annotation(
    Line(points = {{140, -80}, {120, -80}}, color = {0, 127, 255}));
  connect(senVolFlo.port_a, pipe3.port_b) annotation(
    Line(points = {{70, -80}, {100, -80}}, color = {0, 127, 255}));
  connect(prescribedTemperature.T, const.y) annotation(
    Line(points = {{138, 0}, {163, 0}}, color = {0, 0, 127}));
  connect(prescribedTemperature.port, pipe2.heatPort) annotation(
    Line(points = {{120, 0}, {60, 0}, {60, 70}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe4.heatPort) annotation(
    Line(points = {{120, 0}, {-70, 0}, {-70, -70}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe3.heatPort) annotation(
    Line(points = {{120, 0}, {110, 0}, {110, -70}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe1.heatPort) annotation(
    Line(points = {{120, 0}, {-70, 0}, {-70, 70}}, color = {191, 0, 0}));
  connect(consumer.port_b, port_a2) annotation(
    Line(points = {{252, -10}, {252, -80}, {200, -80}}, color = {0, 127, 255}));
  connect(consumer.port_a, port_b1) annotation(
    Line(points = {{252, 10}, {252, 80}, {200, 80}}, color = {0, 127, 255}));
  connect(pipe2.port_b, senT_WMZ1.port_a) annotation(
    Line(points = {{70, 80}, {100, 80}}, color = {0, 127, 255}));
  connect(senT_WMZ1.port_b, senT_b1.port_a) annotation(
    Line(points = {{120, 80}, {140, 80}}, color = {0, 127, 255}));
  connect(senT_WMZ2.port_a, senVolFlo.port_b) annotation(
    Line(points = {{10, -80}, {50, -80}}, color = {0, 127, 255}));
  connect(pipe4.port_a, senT_WMZ2.port_b) annotation(
    Line(points = {{-60, -80}, {-10, -80}}, color = {0, 127, 255}));
  connect(pipe5.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{-120, 0}, {120, 0}}, color = {191, 0, 0}));
  connect(valve.port_1, port_a1) annotation(
    Line(points = {{-140, 80}, {-198, 80}}, color = {0, 127, 255}));
  connect(pipe1.port_a, valve.port_2) annotation(
    Line(points = {{-80, 80}, {-120, 80}}, color = {0, 127, 255}));
  connect(valve.port_3, pipe5.port_b) annotation(
    Line(points = {{-130, 70}, {-130, 10}}, color = {0, 127, 255}));
  connect(ramp.y, valve.y) annotation(
    Line(points = {{-159, 130}, {-131, 130}, {-131, 92}}, color = {0, 0, 127}));
  connect(pipe4.port_b, junc45.ports[1]) annotation(
    Line(points = {{-80, -80}, {-130, -80}}, color = {0, 127, 255}));
  connect(junc45.ports[2], pipe5.port_a) annotation(
    Line(points = {{-130, -80}, {-130, -10}}, color = {0, 127, 255}));
  connect(junc45.ports[3], port_b2) annotation(
    Line(points = {{-130, -80}, {-200, -80}}, color = {0, 127, 255}));
  connect(PT1_WMZ1.u, senT_WMZ1.T) annotation(
    Line(points = {{90, 118}, {90, 108}, {110, 108}, {110, 92}}, color = {0, 0, 127}));
  connect(senT_b1.T, PT1_b1.u) annotation(
    Line(points = {{150, 91}, {150, 117}}, color = {0, 0, 127}));
  connect(PT1_WMZ2.u, senT_WMZ2.T) annotation(
    Line(points = {{0, -118}, {0, -90}}, color = {0, 0, 127}));
  connect(senT_a2.T, PT1_a2.u) annotation(
    Line(points = {{150, -90}, {150, -118}}, color = {0, 0, 127}));
  annotation(
    Icon(coordinateSystem(extent = {{-200, -200}, {200, 200}}), graphics = {Rectangle(origin = {1, -1}, fillColor = {255, 170, 127}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-181, 161}, {181, -161}}), Text(extent = {{-150, 50}, {150, -50}}, textString = "HK TYP B")}),
    Diagram(coordinateSystem(extent = {{-200, -200}, {200, 200}})));
end Heizkreis_TypB;
