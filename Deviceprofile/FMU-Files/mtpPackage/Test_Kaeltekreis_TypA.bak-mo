within mtpPackage;

model Test_Kaeltekreis_TypA
  //-----medium-----
  package Medium = AixLib.Media.Water annotation(
    choicesAllMatching = true);
  
  //-----blocks boundary-----
  AixLib.Fluid.Sources.Boundary_pT boundaryIn(redeclare package Medium = Medium, T = 283, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-90, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AixLib.Fluid.Sources.Boundary_pT boundaryOut(redeclare package Medium = Medium, nPorts = 1) annotation(
    Placement(visible = true, transformation(origin = {-90, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  
  
  //-----block cooling  circuit-----
  mtpPackage.Kaeltekreis_TypA kaeltekreis_TypA(
    redeclare package Medium = Medium, parameterPipe = AixLib.DataBase.Pipes.Copper.Copper_28x1(),  
    m_flow_nominal = 1, 
    energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial, 
    length = 1, 
    Kv = 10, 
    T_amb = 293.155) annotation(
    Placement(visible = true, transformation(origin = {2.84217e-14, -50}, extent = {{-50, -50}, {50, 50}}, rotation = 0)));
  mtpPackage.HydraulicValveBus hydraulicValveBus annotation(
    Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {36, 52}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp valveOpening2(duration = 500, startTime = 180) annotation(
    Placement(visible = true, transformation(origin = {40, 40}, extent = {{-100, 0}, {-80, 20}}, rotation = 0)));

  //-----blocks pump bus-----
  Modelica.Blocks.Sources.Ramp valveOpening1(duration = 500, startTime = 180) annotation(
    Placement(visible = true, transformation(origin = {40, 80}, extent = {{-100, 0}, {-80, 20}}, rotation = 0)));

//-----connections-----  
equation
  connect(kaeltekreis_TypA.port_b1, kaeltekreis_TypA.port_a2) annotation(
    Line(points = {{50, -20}, {80, -20}, {80, -80}, {50, -80}}, color = {0, 127, 255}));
  connect(boundaryIn.ports[1], kaeltekreis_TypA.port_a1) annotation(
    Line(points = {{-80, -20}, {-50, -20}}, color = {0, 127, 255}));
  connect(boundaryOut.ports[1], kaeltekreis_TypA.port_b2) annotation(
    Line(points = {{-80, -80}, {-50, -80}}, color = {0, 127, 255}));
  connect(valveOpening1.y, hydraulicValveBus.valve1Set) annotation(
    Line(points = {{-38, 90}, {0, 90}, {0, 50}}, color = {0, 0, 127}));
  connect(valveOpening2.y, hydraulicValveBus.valve2Set) annotation(
    Line(points = {{-38, 50}, {0, 50}}, color = {0, 0, 127}));
  connect(hydraulicValveBus, kaeltekreis_TypA.hydraulicValveBus) annotation(
    Line(points = {{0, 50}, {0, 10}}, color = {255, 204, 51}, thickness = 0.5));
end Test_Kaeltekreis_TypA;
