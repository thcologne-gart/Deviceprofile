within mtpPackage;

model Heizkreis_TypB_alt "Modell für Heizkreis Typ B-Beimischschaltung"
  extends AixLib.Systems.HydraulicModules.BaseClasses.PartialHydraulicModule;
  //-----parameters-----
  parameter Modelica.Units.SI.Volume vol = 0.0005 "Mixing Volume" annotation(
    Dialog(tab = "Advanced"));
  parameter AixLib.Fluid.Actuators.Valves.Data.GenericThreeWay valveCharacteristic "Valve characteristic of three way valve" annotation(
    choicesAllMatching = true,
    Placement(transformation(extent = {{-120, -120}, {-100, -100}})),
    Dialog(group = "Actuators"));
  //----pipe 1-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe1(redeclare package Medium = Medium, T_start = T_start, allowFlowReversal = allowFlowReversal, energyDynamics = energyDynamics, hCon = hCon, length = length, m_flow_nominal = m_flow_nominal, massDynamics = massDynamics, parameterIso = parameterIso, parameterPipe = parameterPipe, pipeModel = pipeModel, withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {-60, 20}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----valve-----
  AixLib.Fluid.Actuators.Valves.ThreeWayTable mixingValve(redeclare package Medium = Medium, CvData = AixLib.Fluid.Types.CvTypes.Kv, Kv = Kv, T_start = T_start, dpFixed_nominal = {10, 10}, energyDynamics = energyDynamics, flowCharacteristics1 = valveCharacteristic.a_ab, flowCharacteristics3 = valveCharacteristic.b_ab, init = Modelica.Blocks.Types.Init.InitialState, m_flow_nominal = m_flow_nominal, order = 1, portFlowDirection_1 = Modelica.Fluid.Types.PortFlowDirection.Entering, portFlowDirection_2 = Modelica.Fluid.Types.PortFlowDirection.Leaving, portFlowDirection_3 = Modelica.Fluid.Types.PortFlowDirection.Entering, tau = 0.2, y_start = 0) annotation(
    Placement(visible = true, transformation(origin = {-20, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //----pipe 2-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe2(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {10, 20}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----pump-----
  replaceable AixLib.Systems.HydraulicModules.BaseClasses.BasicPumpInterface PumpInterface(redeclare package Medium = Medium, final allowFlowReversal = allowFlowReversal, final m_flow_nominal = m_flow_nominal, T_start = T_start, final energyDynamics = energyDynamics, final massDynamics = massDynamics) "Needs to be redeclared" annotation(
    Placement(visible = true, transformation(origin = {-10, 0}, extent = {{42, 12}, {58, 28}}, rotation = 0)));
  //-----pipe 3-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe3(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {70, 20}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----pipe 4-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe4(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {50, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----pipe 5-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe5(redeclare package Medium = Medium, T_start = T_start, allowFlowReversal = allowFlowReversal, energyDynamics = energyDynamics, hCon = hCon, length = length, m_flow_nominal = m_flow_nominal, massDynamics = massDynamics, parameterIso = parameterIso, parameterPipe = parameterPipe, pipeModel = pipeModel, withConvection = true, withInsulation = true) annotation(
    Placement(visible = true, transformation(origin = {10, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----mixing volume-----
  AixLib.Fluid.MixingVolumes.MixingVolume junc567(redeclare package Medium = Medium, final massDynamics = massDynamics, T_start = T_start, nPorts = 3, final m_flow_nominal = m_flow_nominal, final V = vol, final energyDynamics = energyDynamics) annotation(
    Placement(visible = true, transformation(origin = {-20, -70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----pipe 6-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe6(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {-50, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----pipe 7-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe7(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {-20, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  //connetions
equation
  connect(hydraulicBus.pumpBus, PumpInterface.pumpBus) annotation(
    Line(points = {{0, 100}, {40, 100}, {40, 28}}, color = {255, 204, 51}, thickness = 0.5));
  connect(senT_a1.port_b, pipe1.port_a) annotation(
    Line(points = {{-88, 20}, {-70, 20}}, color = {0, 127, 255}));
  connect(senT_b2.port_a, pipe6.port_b) annotation(
    Line(points = {{-78, -60}, {-60, -60}}, color = {0, 127, 255}));
  connect(pipe2.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{10, 10}, {10, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(mixingValve.y, hydraulicBus.valveSet) annotation(
    Line(points = {{-20, 32}, {-20, 100}, {0, 100}}, color = {0, 0, 127}));
  connect(mixingValve.y_actual, hydraulicBus.valveMea) annotation(
    Line(points = {{-14, 28}, {-14, 100}, {0, 100}}, color = {0, 0, 127}));
  connect(pipe2.port_b, PumpInterface.port_a) annotation(
    Line(points = {{20, 20}, {32, 20}}, color = {0, 127, 255}));
  connect(PumpInterface.port_b, pipe3.port_a) annotation(
    Line(points = {{48, 20}, {60, 20}}, color = {0, 127, 255}));
  connect(prescribedTemperature.port, pipe3.heatPort) annotation(
    Line(points = {{32, -20}, {20, -20}, {20, 0}, {70, 0}, {70, 10}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe7.heatPort) annotation(
    Line(points = {{32, -20}, {-10, -20}}, color = {191, 0, 0}));
  connect(prescribedTemperature.port, pipe6.heatPort) annotation(
    Line(points = {{32, -20}, {20, -20}, {20, -40}, {-50, -40}, {-50, -50}}, color = {191, 0, 0}));
  connect(pipe1.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{-60, 10}, {-60, 0}, {20, 0}, {20, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe5.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{10, -50}, {10, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(senT_a2.port_b, pipe4.port_a) annotation(
    Line(points = {{72, -60}, {60, -60}}, color = {0, 127, 255}));
  connect(pipe4.port_b, pipe5.port_a) annotation(
    Line(points = {{40, -60}, {20, -60}}, color = {0, 127, 255}));
  connect(pipe4.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{50, -50}, {50, -40}, {20, -40}, {20, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe3.port_b, senT_b1.port_a) annotation(
    Line(points = {{80, 20}, {88, 20}}, color = {0, 127, 255}));
  connect(junc567.ports[1], pipe5.port_b) annotation(
    Line(points = {{-20, -60}, {0, -60}}, color = {0, 127, 255}));
  connect(pipe7.port_a, junc567.ports[2]) annotation(
    Line(points = {{-20, -30}, {-20, -60}}, color = {0, 127, 255}));
  connect(junc567.ports[3], pipe6.port_a) annotation(
    Line(points = {{-20, -60}, {-40, -60}}, color = {0, 127, 255}));
  connect(mixingValve.port_3, pipe7.port_b) annotation(
    Line(points = {{-20, 10}, {-20, -10}}, color = {0, 127, 255}));
  connect(mixingValve.port_2, pipe2.port_a) annotation(
    Line(points = {{-10, 20}, {0, 20}}, color = {0, 127, 255}));
  connect(pipe1.port_b, mixingValve.port_1) annotation(
    Line(points = {{-50, 20}, {-30, 20}}, color = {0, 127, 255}));
  annotation(
    Icon);
end Heizkreis_TypB_alt;
