within mtpPackage;

model Heizkreis_TypA_alt "Modell für Heizkreis Typ A-Drosselschaltung"
  extends AixLib.Systems.HydraulicModules.BaseClasses.PartialHydraulicModule;
  //-----parameters-----
  parameter Modelica.Units.SI.Volume vol = 0.0005 "Mixing Volume" annotation(
    Dialog(tab = "Advanced"));
  //----pipe 1-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe1(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {-50, 20}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----pump-----
  replaceable AixLib.Systems.HydraulicModules.BaseClasses.BasicPumpInterface PumpInterface(redeclare package Medium = Medium, final allowFlowReversal = allowFlowReversal, final m_flow_nominal = m_flow_nominal, T_start = T_start, final energyDynamics = energyDynamics, final massDynamics = massDynamics) "Needs to be redeclared" annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{42, 12}, {58, 28}}, rotation = 0)));
  //----pipe 2-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe2(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {50, 20}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----pipe 3-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe3(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {50, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----pipe 4-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe4(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {10, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----valve-----
  AixLib.Fluid.Actuators.Valves.TwoWayTable valve(CvData = AixLib.Fluid.Types.CvTypes.Kv, redeclare package Medium = Medium, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, Kv = Kv, order = 1, init = Modelica.Blocks.Types.Init.InitialState, y_start = 0, flowCharacteristics = AixLib.Fluid.Actuators.Valves.Data.Linear()) annotation(
    Placement(visible = true, transformation(origin = {-20, -60}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  //-----pipe 5-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe5(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {-50, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //connetions
equation
  connect(hydraulicBus.pumpBus, PumpInterface.pumpBus) annotation(
    Line(points = {{0, 100}, {0, 28}}, color = {255, 204, 51}, thickness = 0.5));
  connect(hydraulicBus.valveSet, valve.y) annotation(
    Line(points = {{0, 100}, {-122, 100}, {-122, -90}, {-20, -90}, {-20, -72}}, color = {0, 0, 127}));
  connect(valve.y_actual, hydraulicBus.valveMea) annotation(
    Line(points = {{-24, -66}, {-24, -90}, {-122, -90}, {-122, 100}, {0, 100}}, color = {0, 0, 127}));
  connect(senT_a1.port_b, pipe1.port_a) annotation(
    Line(points = {{-88, 20}, {-60, 20}}, color = {0, 127, 255}));
  connect(PumpInterface.port_b, pipe2.port_a) annotation(
    Line(points = {{8, 20}, {40, 20}}, color = {0, 127, 255}));
  connect(PumpInterface.port_a, pipe1.port_b) annotation(
    Line(points = {{-8, 20}, {-40, 20}}, color = {0, 127, 255}));
  connect(pipe2.port_b, senT_b1.port_a) annotation(
    Line(points = {{60, 20}, {88, 20}}, color = {0, 127, 255}));
  connect(senT_a2.port_b, pipe3.port_a) annotation(
    Line(points = {{72, -60}, {60, -60}}, color = {0, 127, 255}));
  connect(pipe3.port_b, pipe4.port_a) annotation(
    Line(points = {{40, -60}, {20, -60}}, color = {0, 127, 255}));
  connect(pipe4.port_b, valve.port_a) annotation(
    Line(points = {{0, -60}, {-10, -60}}, color = {0, 127, 255}));
  connect(senT_b2.port_a, pipe5.port_b) annotation(
    Line(points = {{-78, -60}, {-60, -60}}, color = {0, 127, 255}));
  connect(pipe5.port_a, valve.port_b) annotation(
    Line(points = {{-40, -60}, {-30, -60}}, color = {0, 127, 255}));
  connect(pipe1.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{-50, 10}, {-50, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe2.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{50, 10}, {50, 0}, {20, 0}, {20, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe5.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{-50, -50}, {-50, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe4.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{10, -50}, {10, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe3.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{50, -50}, {50, -40}, {20, -40}, {20, -20}, {32, -20}}, color = {191, 0, 0}));
  annotation(
    Icon(graphics = {Ellipse(origin = {0, 61}, lineColor = {151, 151, 151}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 10}, {10, -10}}), Line(origin = {4.72927, 60.644}, points = {{-4.64645, 10}, {5.35355, 0}, {-4.64645, -10}, {-4.64645, -10}}, color = {154, 154, 154}, thickness = 0.5), Polygon(origin = {-6, -60}, lineColor = {154, 154, 154}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, points = {{-5, 5}, {5, 0}, {-5, -5}, {-5, 5}, {-5, 5}}), Polygon(origin = {4, -60}, rotation = 180, lineColor = {154, 154, 154}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, points = {{-5, 5}, {5, 0}, {-5, -5}, {-5, 5}, {-5, 5}}), Line(origin = {-0.549451, -63.0055}, points = {{0, 3.5}, {0, -1.5}}, color = {154, 154, 154}, thickness = 0.5), Rectangle(origin = {-1, -68}, lineColor = {154, 154, 154}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-3, 3}, {3, -3}})}));
end Heizkreis_TypA_alt;
