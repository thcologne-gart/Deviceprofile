within mtpPackage;

model Test_RLTTypA1_Sicherung
  mtpPackage.RLTTypA1_Sicherung rlt(
    massDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    hydraulicEfficiency(V_flow={0,1000}, eta={0.7,0.7}),
    preheater(redeclare AixLib.Systems.HydraulicModules.Admix hydraulicModule(
        parameterPipe=AixLib.DataBase.Pipes.Copper.Copper_35x1_5(),
        length=1,
        Kv=6.3,
        valveCharacteristic=
            AixLib.Fluid.Actuators.Valves.Data.LinearEqualPercentage(),
        redeclare
          AixLib.Systems.HydraulicModules.BaseClasses.PumpInterface_SpeedControlledNrpm
          PumpInterface(pump(redeclare
              AixLib.Fluid.Movers.Data.Pumps.Wilo.Stratos25slash1to8 per))),
        dynamicHX(
        dp1_nominal=50,
        dp2_nominal=5000,
        tau1=5,
        tau2=15,
        dT_nom=30,
        Q_nom=30000)),
    redeclare package Medium1 = AixLib.Media.Air,
    redeclare package Medium2 = AixLib.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_amb=293.15,
    m1_flow_nominal=3000/3600*1.2,
    m2_flow_nominal=0.5,
    T_start=293.15,
    usePreheater=true,
    useHumidifierRet=true,
    useHumidifier=true,
    cooler(redeclare AixLib.Systems.HydraulicModules.Admix hydraulicModule(
        parameterPipe=AixLib.DataBase.Pipes.Copper.Copper_35x1_5(),
        length=1,
        Kv=6.3,
        valveCharacteristic=
            AixLib.Fluid.Actuators.Valves.Data.LinearEqualPercentage(),
        redeclare
          AixLib.Systems.HydraulicModules.BaseClasses.PumpInterface_SpeedControlledNrpm
          PumpInterface(pump(redeclare
              AixLib.Fluid.Movers.Data.Pumps.Wilo.Stratos50slash1to12 per))),
        dynamicHX(
        dp1_nominal=80,
        dp2_nominal=1000,
        tau1=5,
        tau2=10,
        dT_nom=30,
        Q_nom=50000)),
    heater(redeclare AixLib.Systems.HydraulicModules.Admix hydraulicModule(
        parameterPipe=AixLib.DataBase.Pipes.Copper.Copper_35x1_5(),
        length=1,
        Kv=6.3,
        valveCharacteristic=
            AixLib.Fluid.Actuators.Valves.Data.LinearEqualPercentage(),
        redeclare
          AixLib.Systems.HydraulicModules.BaseClasses.PumpInterface_SpeedControlledNrpm
          PumpInterface(pump(redeclare
              AixLib.Fluid.Movers.Data.Pumps.Wilo.Stratos25slash1to8 per))),
        dynamicHX(
        dp1_nominal=50,
        dp2_nominal=5000,
        tau1=5,
        tau2=15,
        dT_nom=30,
        Q_nom=30000)),
    dynamicHX(
      dp1_nominal=200,
      dp2_nominal=200,
      dT_nom=2,
      Q_nom=30000),
    humidifier(
      dp_nominal=20,
      mWat_flow_nominal=1,
      TLiqWat_in=288.15),
    humidifierRet(
      dp_nominal=20,
      mWat_flow_nominal=0.5,
      TLiqWat_in=288.15)) annotation(
    Placement(visible = true, transformation(origin = {1.06581e-14, 16}, extent = {{-60, -30}, {60, 36}}, rotation = 0)));
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
  AixLib.Systems.ModularAHU.Controller.CtrAHUBasic ctrAHUBasic(TFlowSet=293.15,
    usePreheater=true,
    useTwoFanCtr=true,
    VFlowSet=3000/3600,
    initType=Modelica.Blocks.Types.Init.InitialState, ctrRh(k=0.01)) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-40, 60}, {-20, 80}}, rotation = 0)));
equation
  connect(boundaryExhaustAir.ports[1], rlt.port_b2) annotation(
    Line(points = {{-70, 40}, {-60, 40}}, color = {0, 127, 255}));
  connect(rlt.port_a1, boundaryOutsideAir.ports[1]) annotation(
    Line(points = {{-60, 16}, {-70, 16}}, color = {0, 127, 255}));
  connect(rlt.port_a3, SourcePreheater.ports[1]) annotation(
    Line(points = {{-44, -14}, {-44, -26}, {-46, -26}, {-46, -32}}, color = {0, 127, 255}));
  connect(rlt.port_b3, SinkPreheater.ports[1]) annotation(
    Line(points = {{-32, -14}, {-32, -26}, {-26, -26}, {-26, -32}}, color = {0, 127, 255}));
  connect(rlt.port_a4, SourceCooler.ports[1]) annotation(
    Line(points = {{0, -14}, {0, -26}, {-2, -26}, {-2, -32}}, color = {0, 127, 255}));
  connect(rlt.port_b4, SinkCooler.ports[1]) annotation(
    Line(points = {{10, -14}, {12, -14}, {12, -26}, {14, -26}, {14, -32}}, color = {0, 127, 255}));
  connect(rlt.port_a5, SourceHeater.ports[1]) annotation(
    Line(points = {{22, -14}, {22, -26}, {32, -26}, {32, -32}}, color = {0, 127, 255}));
  connect(rlt.port_b5, SinkHeater.ports[1]) annotation(
    Line(points = {{32, -14}, {32, -20}, {48, -20}, {48, -32}}, color = {0, 127, 255}));
  connect(rlt.port_a2, boundaryReturnAir.ports[1]) annotation(
    Line(points = {{60, 40}, {70, 40}}, color = {0, 127, 255}));
  connect(rlt.port_b1, boundarySupplyAir.ports[1]) annotation(
    Line(points = {{60, 16}, {70, 16}}, color = {0, 127, 255}));
  connect(ctrAHUBasic.genericAHUBus, rlt.genericAHUBus) annotation(
    Line(points = {{-20, 70}, {0, 70}, {0, 52}}, thickness = 0.5));
end Test_RLTTypA1_Sicherung;
