within mtpPackage;

model Test_Heizkreis_TypA_new
  //-----medium-----
  package Medium = AixLib.Media.Water annotation(
    choicesAllMatching = true);
  
  //-----blocks boundary-----
  AixLib.Fluid.Sources.Boundary_pT boundaryIn(redeclare package Medium = Medium, T = 283, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-90, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sources.Boundary_pT boundaryOut(redeclare package Medium = Medium, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-90, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //-----block heating circuit-----
  //-----blocks pump bus-----
  Modelica.Blocks.Sources.Ramp valveOpening(duration = 500, startTime = 180) annotation(
    Placement(visible = true, transformation(origin = {60, 60}, extent = {{-100, 0}, {-80, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant RPM(k = 2000) annotation(
    Placement(visible = true, transformation(origin = {20, 40}, extent = {{-100, 40}, {-80, 60}}, rotation = 0)));
  AixLib.Systems.HydraulicModules.BaseClasses.HydraulicBus hydraulicBus annotation(
    Placement(visible = true, transformation(origin = {40, 40}, extent = {{-50, 0}, {-30, 20}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-50, 0}, {-30, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanStep SB_pump(startTime = 100, startValue = false) annotation(
    Placement(visible = true, transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  mtpPackage.Heizkreis_TypA_new heizkreis_TypA_new(
  parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(), 
    redeclare AixLib.Systems.HydraulicModules.BaseClasses.PumpInterface_SpeedControlledNrpm PumpInterface(
      pump(
        redeclare AixLib.Fluid.Movers.Data.Pumps.Wilo.Stratos25slash1to6 per)), 
    m_flow_nominal = 1, 
    energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, 
    length = 1, 
    Kv = 10, 
    T_amb = 293.15)
    annotation(
    Placement(visible = true, transformation(origin = {-7, -49}, extent = {{-51, -51}, {51, 51}}, rotation = 0)));
equation
  connect(valveOpening.y, hydraulicBus.valveSet) annotation(
    Line(points = {{-19, 70}, {-9.5, 70}, {-9.5, 50}, {0, 50}}, color = {0, 0, 127}));
  connect(RPM.y, hydraulicBus.pumpBus.rpmSet) annotation(
    Line(points = {{-59, 90}, {0, 90}, {0, 50}}, color = {0, 0, 127}));
  connect(SB_pump.y, hydraulicBus.pumpBus.onSet) annotation(
    Line(points = {{-58, 50}, {0, 50}}, color = {255, 0, 255}));
  connect(boundaryIn.ports[1], heizkreis_TypA_new.port_a1) annotation(
    Line(points = {{-80, -20}, {-70, -20}, {-70, -18}, {-58, -18}}, color = {0, 127, 255}));
  connect(heizkreis_TypA_new.port_b2, boundaryOut.ports[1]) annotation(
    Line(points = {{-58, -80}, {-80, -80}}, color = {0, 127, 255}));
  connect(heizkreis_TypA_new.port_b1, heizkreis_TypA_new.port_a2) annotation(
    Line(points = {{44, -18}, {68, -18}, {68, -80}, {44, -80}}, color = {0, 127, 255}));
  connect(hydraulicBus, heizkreis_TypA_new.hydraulicBus) annotation(
    Line(points = {{0, 50}, {0, 2}, {-8, 2}}, thickness = 0.5));
end Test_Heizkreis_TypA_new;
