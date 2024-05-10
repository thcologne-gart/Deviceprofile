within mtpPackage;

model Heizkreis_TypA_new "Modell für Heizkreis Typ A-Drosselschaltung"
  
  //--------------------
  //media
  //--------------------
  replaceable package Medium1 =
    Modelica.Media.Interfaces.PartialMedium "Medium 1 in the component"
      annotation (choices(
        choice(redeclare package Medium = AixLib.Media.Air "Moist air"),
        choice(redeclare package Medium = AixLib.Media.Water "Water"),
        choice(redeclare package Medium =
            AixLib.Media.Antifreeze.PropyleneGlycolWater (
          property_T=293.15,
          X_a=0.40)
          "Propylene glycol water, 40% mass fraction")));
  replaceable package Medium2 =
    Modelica.Media.Interfaces.PartialMedium "Medium 2 in the component"
      annotation (choices(
        choice(redeclare package Medium = AixLib.Media.Air "Moist air"),
        choice(redeclare package Medium = AixLib.Media.Water "Water"),
        choice(redeclare package Medium =
            AixLib.Media.Antifreeze.PropyleneGlycolWater (
          property_T=293.15,
          X_a=0.40)
          "Propylene glycol water, 40% mass fraction")));
  
  //--------------------
  //parameters
  //--------------------
  parameter Boolean allowFlowReversal1 = true
    "= false to simplify equations, assuming, but not enforcing, no flow reversal for medium 1"
    annotation(Dialog(tab="Assumptions"), Evaluate=true);
  parameter Boolean allowFlowReversal2 = true
    "= false to simplify equations, assuming, but not enforcing, no flow reversal for medium 2"
    annotation(Dialog(tab="Assumptions"), Evaluate=true);
  parameter Modelica.Units.SI.Volume vol = 0.0005 "Mixing Volume" annotation(
    Dialog(tab = "Advanced"));

  //--------------------
  //blocks for ports
  //--------------------
  Modelica.Fluid.Interfaces.FluidPort_a port_a1(
                     redeclare final package Medium = Medium1,
                     m_flow(min=if allowFlowReversal1 then -Modelica.Constants.inf else 0),
                     h_outflow(start = Medium1.h_default, nominal = Medium1.h_default))
    "Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)"
    annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(
                     redeclare final package Medium = Medium1,
                     m_flow(max=if allowFlowReversal1 then +Modelica.Constants.inf else 0),
                     h_outflow(start = Medium1.h_default, nominal = Medium1.h_default))
    "Fluid connector b1 (positive design flow direction is from port_a1 to port_b1)"
    annotation (Placement(transformation(extent={{110,50},{90,70}})));

  Modelica.Fluid.Interfaces.FluidPort_a port_a2(
                     redeclare final package Medium = Medium2,
                     m_flow(min=if allowFlowReversal2 then -Modelica.Constants.inf else 0),
                     h_outflow(start = Medium2.h_default, nominal = Medium2.h_default))
    "Fluid connector a2 (positive design flow direction is from port_a2 to port_b2)"
    annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2(
                     redeclare final package Medium = Medium2,
                     m_flow(max=if allowFlowReversal2 then +Modelica.Constants.inf else 0),
                     h_outflow(start = Medium2.h_default, nominal = Medium2.h_default))
    "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)"
    annotation (Placement(transformation(extent={{-90,-70},{-110,-50}})));
  
  //--------------------
  //medium in system
  //--------------------
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium in the system" annotation(
    choicesAllMatching = true);

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
  // Assumptions
  parameter Boolean allowFlowReversal = true "= true to allow flow reversal, false restricts to design direction (port_a -> port_b)" annotation(
    Dialog(tab = "Assumptions"),
    Evaluate = true);
  // Dynamics
  parameter Modelica.Fluid.Types.Dynamics energyDynamics = Modelica.Fluid.Types.Dynamics.DynamicFreeInitial "Type of energy balance: dynamic (3 initialization options) or steady state" annotation(
    Evaluate = true,
    Dialog(tab = "Dynamics", group = "Equations"));
  parameter Modelica.Fluid.Types.Dynamics massDynamics = Modelica.Fluid.Types.Dynamics.DynamicFreeInitial "Type of mass balance: dynamic (3 initialization options) or steady state" annotation(
    Evaluate = true,
    Dialog(tab = "Dynamics", group = "Equations"));
  AixLib.Systems.HydraulicModules.BaseClasses.HydraulicBus hydraulicBus annotation(
    Placement(transformation(extent = {{-20, 80}, {20, 120}}), iconTransformation(extent = {{-20, 80}, {20, 120}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature annotation(
    Placement(transformation(extent = {{-8, -8}, {8, 8}}, rotation = 180, origin = {40, -20})));
  Modelica.Blocks.Sources.Constant const(k = T_amb) annotation(
    Placement(transformation(extent = {{72, -28}, {56, -12}})));
  
  //--------------------
  //blocks for sensors
  //--------------------
  AixLib.Systems.HydraulicModules.SimpleConsumer simpleConsumer annotation(
    Placement(visible = true, transformation(origin = {150, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
protected
  AixLib.Fluid.Sensors.VolumeFlowRate WMZ(redeclare package Medium = Medium, final m_flow_nominal = m_flow_nominal, T_start = T_start, final allowFlowReversal = allowFlowReversal) "Outflow out of forward line" annotation(
    Placement(visible = true, transformation(origin = {34, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senT_a1(tau = 0.01, T_start = T_start, redeclare package Medium = Medium, transferHeat = true, final TAmb = T_amb, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, tauHeaTra = tauHeaTra) annotation(
    Placement(transformation(extent = {{-100, 14}, {-88, 26}})));
  AixLib.Fluid.Sensors.TemperatureTwoPort senT_a2(redeclare package Medium = Medium, tau = 0.01, transferHeat = true, final TAmb = T_amb, final m_flow_nominal = m_flow_nominal, T_start = T_start, final allowFlowReversal = allowFlowReversal, tauHeaTra = tauHeaTra) annotation(
    Placement(visible = true, transformation(origin = {2, 0}, extent = {{84, -66}, {72, -54}}, rotation = 0)));
  AixLib.Fluid.Sensors.TemperatureTwoPort senT_b1(final m_flow_nominal = m_flow_nominal, tau = 0.01, T_start = T_start, redeclare package Medium = Medium, transferHeat = true, final TAmb = T_amb, final allowFlowReversal = allowFlowReversal, tauHeaTra = tauHeaTra) annotation(
    Placement(transformation(extent = {{88, 14}, {100, 26}})));
  AixLib.Fluid.Sensors.TemperatureTwoPort senT_b2(tau = 0.01, T_start = T_start, redeclare package Medium = Medium, transferHeat = true, final TAmb = T_amb, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, tauHeaTra = tauHeaTra) annotation(
    Placement(visible = true, transformation(origin = {68, 0}, extent = {{-78, -66}, {-90, -54}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder PT1_b2(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(visible = true, transformation(origin = {-110, -30}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder PT1_b1(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 270, origin = {70, 70})));
  Modelica.Blocks.Continuous.FirstOrder PT1_a1(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 270, origin = {-70, 70})));
  Modelica.Blocks.Continuous.FirstOrder PT1_a2(initType = Modelica.Blocks.Types.Init.SteadyState, y_start = T_start, final T = tau) annotation(
    Placement(transformation(extent = {{10, 10}, {-10, -10}}, rotation = 180, origin = {110, -30})));

  
  //----pipe 1-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe1(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {-70, 20}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----pump-----
  replaceable AixLib.Systems.HydraulicModules.BaseClasses.BasicPumpInterface PumpInterface(redeclare package Medium = Medium, final allowFlowReversal = allowFlowReversal, final m_flow_nominal = m_flow_nominal, T_start = T_start, final energyDynamics = energyDynamics, final massDynamics = massDynamics) "Needs to be redeclared" annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{42, 12}, {58, 28}}, rotation = 0)));
  //----pipe 2-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe2(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {60, 20}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  //-----pipe 3-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe3(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {60, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----pipe 4-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe4(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {10, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  //-----valve-----
  AixLib.Fluid.Actuators.Valves.TwoWayTable valve(CvData = AixLib.Fluid.Types.CvTypes.Kv, redeclare package Medium = Medium, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, Kv = Kv, order = 1, init = Modelica.Blocks.Types.Init.InitialState, y_start = 0, flowCharacteristics = AixLib.Fluid.Actuators.Valves.Data.Linear()) annotation(
    Placement(visible = true, transformation(origin = {-40, -60}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  //-----pipe 5-----
  AixLib.Fluid.FixedResistances.GenericPipe pipe5(redeclare package Medium = Medium, pipeModel = pipeModel, T_start = T_start, final m_flow_nominal = m_flow_nominal, final allowFlowReversal = allowFlowReversal, length = length, parameterPipe = parameterPipe, parameterIso = parameterIso, final hCon = hCon, final energyDynamics = energyDynamics, final massDynamics = massDynamics) annotation(
    Placement(visible = true, transformation(origin = {-70, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  
  //--------------------
  //connetions
  //--------------------
equation
  connect(hydraulicBus.pumpBus, PumpInterface.pumpBus) annotation(
    Line(points = {{0, 100}, {0, 28}}, color = {255, 204, 51}, thickness = 0.5));
  connect(hydraulicBus.valveSet, valve.y) annotation(
    Line(points = {{0, 100}, {-122, 100}, {-122, -90}, {-40, -90}, {-40, -72}}, color = {0, 0, 127}));
  connect(valve.y_actual, hydraulicBus.valveMea) annotation(
    Line(points = {{-45, -67}, {-45, -90}, {-122, -90}, {-122, 100}, {0, 100}}, color = {0, 0, 127}));
  connect(senT_a1.port_b, pipe1.port_a) annotation(
    Line(points = {{-88, 20}, {-80, 20}}, color = {0, 127, 255}));
  connect(PumpInterface.port_b, pipe2.port_a) annotation(
    Line(points = {{8, 20}, {50, 20}}, color = {0, 127, 255}));
  connect(PumpInterface.port_a, pipe1.port_b) annotation(
    Line(points = {{-8, 20}, {-60, 20}}, color = {0, 127, 255}));
  connect(pipe2.port_b, senT_b1.port_a) annotation(
    Line(points = {{70, 20}, {88, 20}}, color = {0, 127, 255}));
  connect(senT_a2.port_b, pipe3.port_a) annotation(
    Line(points = {{74, -60}, {70, -60}}, color = {0, 127, 255}));
  connect(pipe5.port_a, valve.port_b) annotation(
    Line(points = {{-60, -60}, {-50, -60}}, color = {0, 127, 255}));
  connect(pipe1.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{-70, 10}, {-70, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe2.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{60, 10}, {60, 0}, {20, 0}, {20, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe5.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{-70, -50}, {-70, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe4.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{10, -50}, {10, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(pipe3.heatPort, prescribedTemperature.port) annotation(
    Line(points = {{60, -50}, {60, -40}, {20, -40}, {20, -20}, {32, -20}}, color = {191, 0, 0}));
  connect(senT_a1.port_a, port_a1) annotation(
    Line(points = {{-100, 20}, {-100, 60}}, color = {0, 127, 255}));
  connect(senT_a2.port_a, port_a2) annotation(
    Line(points = {{86, -60}, {100, -60}}, color = {0, 127, 255}));
  connect(WMZ.port_b, pipe4.port_a) annotation(
    Line(points = {{24, -60}, {20, -60}}, color = {0, 127, 255}));
  connect(WMZ.port_a, pipe3.port_b) annotation(
    Line(points = {{44, -60}, {50, -60}}, color = {0, 127, 255}));
  connect(pipe5.port_b, port_b2) annotation(
    Line(points = {{-80, -60}, {-100, -60}}, color = {0, 127, 255}));
  connect(senT_b2.port_b, valve.port_a) annotation(
    Line(points = {{-22, -60}, {-30, -60}}, color = {0, 127, 255}));
  connect(pipe4.port_b, senT_b2.port_a) annotation(
    Line(points = {{0, -60}, {-10, -60}}));
  connect(senT_a1.T, PT1_a1.u) annotation(
    Line(points = {{-94, 26}, {-94, 40}, {-70, 40}, {-70, 58}}, color = {0, 0, 127}));
  connect(senT_b1.T, PT1_b1.u) annotation(
    Line(points = {{94, 26}, {94, 46}, {70, 46}, {70, 58}}, color = {0, 0, 127}));
  connect(PT1_a2.u, senT_a2.T) annotation(
    Line(points = {{98, -30}, {80, -30}, {80, -54}}, color = {0, 0, 127}));
  connect(PT1_b2.u, senT_b2.T) annotation(
    Line(points = {{-98, -30}, {-16, -30}, {-16, -54}}, color = {0, 0, 127}));
  connect(const.y, prescribedTemperature.T) annotation(
    Line(points = {{56, -20}, {50, -20}}, color = {0, 0, 127}));
  connect(PT1_a1.y, hydraulicBus.TFwrdInMea) annotation(
    Line(points = {{-70, 82}, {-70, 100}, {0, 100}}, color = {0, 0, 127}));
  connect(PT1_b1.y, hydraulicBus.TFwrdOutMea) annotation(
    Line(points = {{70, 82}, {70, 100}, {0, 100}}, color = {0, 0, 127}));
  connect(PT1_a2.y, hydraulicBus.TRtrnInMea) annotation(
    Line(points = {{122, -30}, {124, -30}, {124, 100}, {0, 100}}, color = {0, 0, 127}));
  connect(PT1_b2.y, hydraulicBus.TRtrnOutMea) annotation(
    Line(points = {{-120, -30}, {-122, -30}, {-122, 100}, {0, 100}}, color = {0, 0, 127}));
  connect(senT_b1.port_b, port_b1) annotation(
    Line(points = {{100, 20}, {100, 60}}, color = {0, 127, 255}));
  connect(simpleConsumer.port_a, port_b1) annotation(
    Line(points = {{150, 10}, {150, 60}, {100, 60}}, color = {0, 127, 255}));
  connect(simpleConsumer.port_b, port_a2) annotation(
    Line(points = {{150, -10}, {150, -60}, {100, -60}}, color = {0, 127, 255}));
  annotation(
    Icon(graphics = {Ellipse(origin = {0, 61}, lineColor = {151, 151, 151}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 10}, {10, -10}}), Line(origin = {4.72927, 60.644}, points = {{-4.64645, 10}, {5.35355, 0}, {-4.64645, -10}, {-4.64645, -10}}, color = {154, 154, 154}, thickness = 0.5), Polygon(origin = {-6, -60}, lineColor = {154, 154, 154}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, points = {{-5, 5}, {5, 0}, {-5, -5}, {-5, 5}, {-5, 5}}), Polygon(origin = {4, -60}, rotation = 180, lineColor = {154, 154, 154}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, points = {{-5, 5}, {5, 0}, {-5, -5}, {-5, 5}, {-5, 5}}), Line(origin = {-0.549451, -63.0055}, points = {{0, 3.5}, {0, -1.5}}, color = {154, 154, 154}, thickness = 0.5), Rectangle(origin = {-1, -68}, lineColor = {154, 154, 154}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-3, 3}, {3, -3}})}));
end Heizkreis_TypA_new;
