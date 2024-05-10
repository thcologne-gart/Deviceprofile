within mtpPackage;

model Heizkreis_TypAA "Modell für Heizkreis Typ A-Drosselschaltung"
  package Medium = AixLib.Media.Water "Medium 1 in the component";
  package Medium1 = AixLib.Media.Water "Medium 1 in the component";
  package Medium2 = AixLib.Media.Water "Medium 1 in the component";
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
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature annotation(
    Placement(visible = true, transformation(origin = {40, 0}, extent = {{-8, -8}, {8, 8}}, rotation = 180)));
  Modelica.Blocks.Sources.Constant const(k = T_amb) annotation(
    Placement(visible = true, transformation(origin = {0, 20}, extent = {{72, -28}, {56, -12}}, rotation = 0)));
  //-----parameters-----
  parameter Modelica.Units.SI.Volume vol = 0.0005 "Mixing Volume" annotation(
    Dialog(tab = "Advanced"));
  //----pipe 1-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe1(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {-50, 80}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----pump-----
  AixLib.Fluid.Movers.SpeedControlled_y pump(redeclare package Medium = Medium, redeclare AixLib.Fluid.Movers.Data.Pumps.Wilo.Stratos25slash1to4 per(speeds_rpm = 1800*{0, 0.5, 1}, constantSpeed_rpm = 1800), use_inputFilter = false, inputType = AixLib.Fluid.Types.InputType.Constant, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState) annotation(
    Placement(visible = true, transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //----pipe 2-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe2(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {50, 80}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----pipe 3-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe3(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {50, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----pipe 4-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe4(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {10, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----valve-----
  AixLib.Fluid.Actuators.Valves.TwoWayTable valve(redeclare package Medium = Medium,
    use_inputFilter=false,
    from_dp=true,
    flowCharacteristics=datVal,
    CvData=AixLib.Fluid.Types.CvTypes.Kv,
    Kv=0.65,
    m_flow_nominal=0.03)
    "Valve model with opening characteristics based on a table" annotation(
    Placement(visible = true, transformation(origin = {-20, -80}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  parameter AixLib.Fluid.Actuators.Valves.Data.Generic datVal(
    y={0,0.1667,0.3333,0.5,0.6667,1},
    phi={0, 0.19, 0.35, 0.45, 0.5, 0.65}/0.65) "Valve characteristics"
    annotation (Placement(transformation(extent={{60,60},{80,80}})));  
  //-----pipe 5-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe5(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 10, m_flow_nominal = 0.3, nNodes = 3, parameterIso = AixLib.DataBase.Pipes.Insulation.Iso50pc(), parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), pipeModel = "SimplePipe", withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {-50, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //connetions
  Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare final package Medium = Medium1, m_flow(min = if allowFlowReversal1 then -Modelica.Constants.inf else 0), h_outflow(start = Medium1.h_default, nominal = Medium1.h_default)) "Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)" annotation(
    Placement(visible = true, transformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-102, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(redeclare final package Medium = Medium1, m_flow(max = if allowFlowReversal1 then +Modelica.Constants.inf else 0), h_outflow(start = Medium1.h_default, nominal = Medium1.h_default)) "Fluid connector b1 (positive design flow direction is from port_a1 to port_b1)" annotation(
    Placement(visible = true, transformation(origin = {100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_a port_a2(redeclare final package Medium = Medium2, m_flow(min = if allowFlowReversal2 then -Modelica.Constants.inf else 0), h_outflow(start = Medium2.h_default, nominal = Medium2.h_default)) "Fluid connector a2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2(redeclare final package Medium = Medium2, m_flow(max = if allowFlowReversal2 then +Modelica.Constants.inf else 0), h_outflow(start = Medium2.h_default, nominal = Medium2.h_default)) "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)" annotation(
    Placement(visible = true, transformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-102, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sources.Boundary_pT boundaryIn(redeclare package Medium = Medium, T = 283, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-150, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sources.Boundary_pT boundaryOut(redeclare package Medium = Medium, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-150, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp valveOpening(duration = 500, startTime = 180) annotation(
    Placement(visible = true, transformation(origin = {40, -140}, extent = {{-100, 0}, {-80, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant RPM(k = 1) annotation(
    Placement(visible = true, transformation(origin = {60, 80}, extent = {{-100, 40}, {-80, 60}}, rotation = 0)));
equation
  connect(pipe3.port_b, pipe4.port_a) annotation(
    Line(points = {{40, -80}, {20, -80}}, color = {0, 127, 255}));
  connect(pipe4.port_b, valve.port_a) annotation(
    Line(points = {{0, -80}, {-10, -80}}, color = {0, 127, 255}));
  connect(pipe1.port_a, port_a1) annotation(
    Line(points = {{-60, 80}, {-100, 80}}, color = {0, 127, 255}));
  connect(pipe3.port_a, port_a2) annotation(
    Line(points = {{60, -80}, {100, -80}}, color = {0, 127, 255}));
  connect(pipe2.port_b, port_b1) annotation(
    Line(points = {{60, 80}, {100, 80}}, color = {0, 127, 255}));
  connect(prescribedTemperature.T, const.y) annotation(
    Line(points = {{49.6, 0}, {55.6, 0}}, color = {0, 0, 127}));
  connect(pipe5.port_b, port_b2) annotation(
    Line(points = {{-60, -80}, {-100, -80}}, color = {0, 127, 255}));
  connect(pipe5.port_a, valve.port_b) annotation(
    Line(points = {{-40, -80}, {-30, -80}}, color = {0, 127, 255}));
  connect(pipe1.port_b, pump.port_a) annotation(
    Line(points = {{-40, 80}, {-10, 80}}));
  connect(pump.port_b, pipe2.port_a) annotation(
    Line(points = {{10, 80}, {40, 80}}));
  connect(boundaryIn.ports[1], port_a1) annotation(
    Line(points = {{-140, 80}, {-100, 80}}, color = {0, 127, 255}));
  connect(boundaryOut.ports[1], port_b2) annotation(
    Line(points = {{-140, -80}, {-100, -80}}, color = {0, 127, 255}));
  connect(prescribedTemperature.port, pump.heatPort) annotation(
    Line(points = {{32, 0}, {0, 0}, {0, 74}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe1.heatPort) annotation(
    Line(points = {{32, 0}, {-50, 0}, {-50, 70}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe4.heatPort) annotation(
    Line(points = {{32, 0}, {10, 0}, {10, -70}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe5.heatPort) annotation(
    Line(points = {{32, 0}, {-50, 0}, {-50, -70}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe2.heatPort) annotation(
    Line(points = {{32, 0}, {20, 0}, {20, 40}, {50, 40}, {50, 70}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe3.heatPort) annotation(
    Line(points = {{32, 0}, {20, 0}, {20, -40}, {50, -40}, {50, -70}}, color = {191, 0, 0}));
  connect(port_b1, port_a2) annotation(
    Line(points = {{100, 80}, {148, 80}, {148, -80}, {100, -80}}));
  connect(valve.y, valveOpening.y) annotation(
    Line(points = {{-20, -92}, {-20, -130}, {-38, -130}}, color = {0, 0, 127}));
  connect(RPM.y, pump.y) annotation(
    Line(points = {{-18, 130}, {0, 130}, {0, 92}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Ellipse(origin = {0, 61}, lineColor = {151, 151, 151}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 10}, {10, -10}}), Line(origin = {4.72927, 60.644}, points = {{-4.64645, 10}, {5.35355, 0}, {-4.64645, -10}, {-4.64645, -10}}, color = {154, 154, 154}, thickness = 0.5), Polygon(origin = {-6, -60}, lineColor = {154, 154, 154}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, points = {{-5, 5}, {5, 0}, {-5, -5}, {-5, 5}, {-5, 5}}), Polygon(origin = {4, -60}, rotation = 180, lineColor = {154, 154, 154}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, points = {{-5, 5}, {5, 0}, {-5, -5}, {-5, 5}, {-5, 5}}), Line(origin = {-0.549451, -63.0055}, points = {{0, 3.5}, {0, -1.5}}, color = {154, 154, 154}, thickness = 0.5), Rectangle(origin = {-1, -68}, lineColor = {154, 154, 154}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-3, 3}, {3, -3}})}));
end Heizkreis_TypAA;
