within mtpPackage;

model Test_Kaeltekreis_TypC
  //-----medium-----
  package Medium = AixLib.Media.Water annotation(
    choicesAllMatching = true);
  //-----blocks boundary-----
  AixLib.Fluid.Sources.Boundary_pT boundaryIn(redeclare package Medium = Medium, T = 283, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-90, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sources.Boundary_pT boundaryOut(redeclare package Medium = Medium, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-90, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //-----block cooling circuit-----
  //-----blocks pump bus-----
  Modelica.Blocks.Sources.Ramp valveOpening(duration = 500, startTime = 180) annotation(
    Placement(visible = true, transformation(origin = {60, 60}, extent = {{-100, 0}, {-80, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant RPM(k = 2000) annotation(
    Placement(visible = true, transformation(origin = {20, 40}, extent = {{-100, 40}, {-80, 60}}, rotation = 0)));
  AixLib.Systems.HydraulicModules.BaseClasses.HydraulicBus hydraulicBus annotation(
    Placement(visible = true, transformation(origin = {40, 40}, extent = {{-50, 0}, {-30, 20}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-50, 0}, {-30, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanStep SB_pump(startTime = 100, startValue = false) annotation(
    Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //-----connections-----
  mtpPackage.Kaeltekreis_TypC kaeltekreis_TypC(redeclare package Medium = AixLib.Media.Water, parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_35x1_5(), valveCharacteristic = AixLib.Fluid.Actuators.Valves.Data.LinearEqualPercentage(), m_flow_nominal = 0.1, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, length = 1, Kv = 10, T_amb = 293.15) annotation(
    Placement(visible = true, transformation(origin = {0, -50}, extent = {{-50, -50}, {50, 50}}, rotation = 0)));
equation
  connect(valveOpening.y, hydraulicBus.valveSet) annotation(
    Line(points = {{-19, 70}, {-9.5, 70}, {-9.5, 50}, {0, 50}}, color = {0, 0, 127}));
  connect(RPM.y, hydraulicBus.pumpBus.rpmSet) annotation(
    Line(points = {{-59, 90}, {0, 90}, {0, 50}}, color = {0, 0, 127}));
  connect(SB_pump.y, hydraulicBus.pumpBus.onSet) annotation(
    Line(points = {{-58, 50}, {0, 50}}, color = {255, 0, 255}));
  connect(kaeltekreis_TypC.port_b1, kaeltekreis_TypC.port_a2) annotation(
    Line(points = {{50, -20}, {80, -20}, {80, -80}, {50, -80}}, color = {0, 127, 255}));
  connect(kaeltekreis_TypC.port_b2, boundaryOut.ports[1]) annotation(
    Line(points = {{-50, -80}, {-80, -80}}, color = {0, 127, 255}));
  connect(boundaryIn.ports[1], kaeltekreis_TypC.port_a1) annotation(
    Line(points = {{-80, -20}, {-50, -20}}, color = {0, 127, 255}));
  connect(hydraulicBus, kaeltekreis_TypC.hydraulicBus) annotation(
    Line(points = {{0, 50}, {0, 0}}, color = {255, 204, 51}, thickness = 0.5));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "dassl"));
end Test_Kaeltekreis_TypC;
