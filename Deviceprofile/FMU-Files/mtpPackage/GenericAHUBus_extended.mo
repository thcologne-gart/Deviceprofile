within mtpPackage;

expandable connector GenericAHUBus_extended "Data bus for generic air-handling unit"
  extends Modelica.Icons.SignalBus;
  import Modelica.Units.SI;
  AixLib.Systems.ModularAHU.BaseClasses.RegisterBus preheaterBus;
  AixLib.Systems.ModularAHU.BaseClasses.RegisterBus coolerBus;
  AixLib.Systems.ModularAHU.BaseClasses.RegisterBus heaterBus;
  AixLib.Fluid.Movers.PumpsPolynomialBased.BaseClasses.PumpBus kvsPumpBus;
  SI.Temperature TOdaMea "Outside air temperature";
  SI.Temperature TSupMea "Supply air temperature";
  SI.Temperature TEtaMea "Extract air temperature";
  SI.Temperature TEhaMea "Exhaust air temperature";
  SI.VolumeFlowRate V_flow_EtaMea "extract air volume flow";
  Real dpFanEtaSet "Set pressure difference for fan in extract air canal";
  Real dpFanEtaMea "Measured pressure difference for fan in extract air canal";
  Real powerFanRetMea "Power of fan in extract air canal";
  Real dpFanSupSet "Set pressure difference for fan in supply air canal";
  Real dpFanSupMea "Measured pressure difference for fan in supply air canal";
  Real powerFanSupMea "Power of fan in supply air canal";
  Real flapOdaSet(start = 1) "Flap opening of flap in outdoor air canal [0..1]";
  Real flapOdaMea "Actual flap opening of flap in extract air canal";
  Real flapEhaSet(start = 1) "Flap opening of flap in exhaust air canal [0..1]";
  Real flapEhaMea "Actual flap opening of flap in exhaust air canal";  
  Real flapEtaSet(start = 1) "Flap opening of flap in extract air canal [0..1]";
  Real flapEtaMea "Actual flap opening of flap in extract air canal";
  Real flapSupSet(start = 1) "Flap opening of flap in supply air canal [0..1]";
  Real flapSupMea "Actual flap opening of flap in supply air canal";
  Real bypassHrsSet "Flap opening of bypass of heat recovery system [0..1]";
  Real bypassHrsMea "Actual flap opening of bypass of heat recovery system";
  Real humSetSup "Set value for humidifier in supply air canal [0..1]";
  Real powerHumSupMea "Consumed power of humidifier in supply air canal [0..1]";
  Real humSetEta "Set value for humidifier in extract air canal [0..1]";
  Real powerHumEtaMea "Consumed power of humidifier in extract air canal [0..1]";
  Real adiabHumSet "Set value for adiabatic humidifier [0..1]";
  Real relHumSupMea "Relative humidity of supply air";
  Real relHumEtaMea "Relative humidity of extract air";
  Real kvsValveSet;
  Real kvsValveMea;
  Real kvsPumpOnSet;
  annotation(
    Icon(graphics, coordinateSystem(preserveAspectRatio = false)),
    Diagram(graphics, coordinateSystem(preserveAspectRatio = false)),
    Documentation(info = "<html>
<p>Definition of a standard bus connector for ahu register modules. The bus connector includes the <a href=\"modelica://AixLib/Systems/HydraulicModules/BaseClasses/HydraulicBus.mo\">HydraulicBus</a>.</p>
</html>", revisions = "<html>
<ul>
<li>October 29, 2019, by Alexander Kümpel:<br/>First implementation. </li>
</ul>
</html>"));
end GenericAHUBus_extended;
