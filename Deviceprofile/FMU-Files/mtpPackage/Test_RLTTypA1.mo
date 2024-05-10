within mtpPackage;

model Test_RLTTypA1
  AixLib.Fluid.Sources.Boundary_pT boundaryReturnAir(T=294.15,
    nPorts=1,
    redeclare package Medium = AixLib.Media.Air) annotation(
    Placement(visible = true, transformation(origin = {80, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.Sources.Boundary_pT boundarySupplyAir(nPorts=1, redeclare package
      Medium = AixLib.Media.Air) annotation(
    Placement(visible = true, transformation(origin = {80, 16}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.Sources.Boundary_pT SourcePreheater(nPorts=1,
    redeclare package Medium = AixLib.Media.Water,
    T=333.15) annotation(
    Placement(visible = true, transformation(origin = {-46, -40}, extent = {{8, -8}, {-8, 8}}, rotation = 270)));
  AixLib.Fluid.Sources.Boundary_pT SinkHeater(nPorts=1, redeclare package Medium =
        AixLib.Media.Water) annotation(
    Placement(visible = true, transformation(origin = {48, -40}, extent = {{8, -8}, {-8, 8}}, rotation = 270)));
  AixLib.Fluid.Sources.Boundary_pT SinkPreheater(nPorts=1,
    redeclare package Medium = AixLib.Media.Water,
    p=102000,
    T=283.15) annotation(
    Placement(visible = true, transformation(origin = {-26, -40}, extent = {{8, -8}, {-8, 8}}, rotation = 270)));
  AixLib.Fluid.Sources.Boundary_pT SourceHeater(nPorts=1,
    redeclare package Medium = AixLib.Media.Water,
    T=333.15) annotation(
    Placement(visible = true, transformation(origin = {32, -40}, extent = {{8, -8}, {-8, 8}}, rotation = 270)));
  AixLib.Fluid.Sources.Boundary_pT SinkCooler(nPorts=1,
    redeclare package Medium = AixLib.Media.Water,
    T=283.15) annotation(
    Placement(visible = true, transformation(origin = {14, -40}, extent = {{8, -8}, {-8, 8}}, rotation = 270)));
  AixLib.Fluid.Sources.Boundary_pT SourceCooler(nPorts=1,
    redeclare package Medium = AixLib.Media.Water,
    T=283.15) annotation(
    Placement(visible = true, transformation(origin = {-2, -40}, extent = {{8, -8}, {-8, 8}}, rotation = 270)));
  AixLib.Fluid.Sources.Boundary_pT boundaryExhaustAir(nPorts=1, redeclare package
      Medium = AixLib.Media.Air) annotation(
    Placement(visible = true, transformation(origin = {-80, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
  AixLib.Fluid.Sources.Boundary_pT boundaryOutsideAir(nPorts=1,
    redeclare package Medium = AixLib.Media.Air,
    T=283.15) annotation(
    Placement(visible = true, transformation(origin = {-80, 16}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
  RLTTypA1 rLTTypA1(massDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    hydraulicEfficiency(V_flow={0,1000}, eta={0.7,0.7}),
    preHeater(
        dp1_nominal=50,
        dp2_nominal=5000,
        tau1=5,
        tau2=15,
        dT_nom=30,
        Q_nom=30000),
    redeclare package Medium1 = AixLib.Media.Air,
    redeclare package Medium2 = AixLib.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_amb=293.15,
    m1_flow_nominal=3000/3600*1.2,
    m2_flow_nominal=0.5,
    T_start=293.15,
    usePreheater=true,
    useHumidifierRet=false,
    useHumidifier=true,
    cooler(
        dp1_nominal=80,
        dp2_nominal=1000,
        tau1=5,
        tau2=10,
        dT_nom=30,
        Q_nom=50000),
    reHeater(
        dp1_nominal=50,
        dp2_nominal=5000,
        tau1=5,
        tau2=15,
        dT_nom=30,
        Q_nom=30000),
    dynamicHX(
      dp1_nominal=200,
      dp2_nominal=200,
      dT_nom=2,
      Q_nom=30000),
    humidifier(
      dp_nominal=20,
      mWat_flow_nominal=1,
      TLiqWat_in=288.15)
      ) annotation(
    Placement(visible = true, transformation(origin = {1.06581e-14, 16}, extent = {{-60, -30}, {60, 36}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant ConstFlap(k = 1) annotation(
    Placement(visible = true, transformation(origin = {-56, 124}, extent = {{20, -20}, {32, -8}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 100) annotation(
    Placement(visible = true, transformation(origin = {-58, 154}, extent = {{20, -20}, {32, -8}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = 0) annotation(
    Placement(visible = true, transformation(origin = {16, 128}, extent = {{20, -20}, {32, -8}}, rotation = 0)));
equation
  connect(rLTTypA1.port_b2, boundaryExhaustAir.ports[1]) annotation(
    Line(points = {{-60, 40}, {-70, 40}}, color = {0, 127, 255}));
  connect(boundaryOutsideAir.ports[1], rLTTypA1.port_a1) annotation(
    Line(points = {{-70, 16}, {-60, 16}}, color = {0, 127, 255}));
  connect(rLTTypA1.port_a3, SourcePreheater.ports[1]) annotation(
    Line(points = {{-44, -14}, {-46, -14}, {-46, -32}}, color = {0, 127, 255}));
  connect(rLTTypA1.port_b3, SinkPreheater.ports[1]) annotation(
    Line(points = {{-32, -14}, {-26, -14}, {-26, -32}}, color = {0, 127, 255}));
  connect(rLTTypA1.port_a4, SourceCooler.ports[1]) annotation(
    Line(points = {{0, -14}, {-2, -14}, {-2, -32}}, color = {0, 127, 255}));
  connect(rLTTypA1.port_b4, SinkCooler.ports[1]) annotation(
    Line(points = {{10, -14}, {14, -14}, {14, -32}}, color = {0, 127, 255}));
  connect(rLTTypA1.port_a5, SourceHeater.ports[1]) annotation(
    Line(points = {{22, -14}, {22, -24}, {32, -24}, {32, -32}}, color = {0, 127, 255}));
  connect(rLTTypA1.port_b5, SinkHeater.ports[1]) annotation(
    Line(points = {{32, -14}, {48, -14}, {48, -32}}, color = {0, 127, 255}));
  connect(rLTTypA1.port_a2, boundaryReturnAir.ports[1]) annotation(
    Line(points = {{60, 40}, {70, 40}}, color = {0, 127, 255}));
  connect(rLTTypA1.port_b1, boundarySupplyAir.ports[1]) annotation(
    Line(points = {{60, 16}, {70, 16}}, color = {0, 127, 255}));
  connect(ConstFlap.y, rLTTypA1.genericAHUBus_extended.flapEhaSet) annotation(
    Line(points = {{-24, 110}, {0, 110}, {0, 62}}, color = {0, 0, 127}));
  connect(ConstFlap.y, rLTTypA1.genericAHUBus_extended.flapEtaSet) annotation(
    Line(points = {{-24, 110}, {0, 110}, {0, 62}}, color = {0, 0, 127}));
  connect(ConstFlap.y, rLTTypA1.genericAHUBus_extended.flapOdaSet) annotation(
    Line(points = {{-24, 110}, {0, 110}, {0, 62}}, color = {0, 0, 127}));
  connect(ConstFlap.y, rLTTypA1.genericAHUBus_extended.flapSupSet) annotation(
    Line(points = {{-24, 110}, {0, 110}, {0, 62}}, color = {0, 0, 127}));
  connect(constant1.y, rLTTypA1.genericAHUBus_extended.dpFanEtaSet) annotation(
    Line(points = {{-26, 140}, {0, 140}, {0, 62}}, color = {0, 0, 127}));
  connect(constant1.y, rLTTypA1.genericAHUBus_extended.dpFanSupSet) annotation(
    Line(points = {{-26, 140}, {0, 140}, {0, 62}}, color = {0, 0, 127}));
  connect(constant2.y, rLTTypA1.genericAHUBus_extended.humSetSup) annotation(
    Line(points = {{48, 114}, {52, 114}, {52, 62}, {0, 62}}, color = {0, 0, 127}));
end Test_RLTTypA1;
