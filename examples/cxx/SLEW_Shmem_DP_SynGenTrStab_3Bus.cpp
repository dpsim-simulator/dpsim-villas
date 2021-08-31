#include <DPsim.h>
#include <dpsim-villas/InterfaceShmem.h>
#include <../../Examples/Cxx/Examples.h>

using namespace DPsim;
using namespace CPS;
using namespace CIM::Examples::Grids::ThreeBus;

ScenarioConfig ThreeBus;

/*
 * This SLEW example runs the DP simulation of the three bus system.
 * The simulation is configured according to the json file.
 * Besides it runs in combination with villas-node to import and export.
 */

int main(int argc, char** argv) {

	// Reads the simulation configuration from the json file
	// provided by --params=<my-json-file>
	fs::path configFilename;
	CommandLineArgs args(argc, argv);
	configFilename = args.params;
	std::ifstream jsonFile(configFilename);
	json simConfig = json::parse(jsonFile);

	// Extract information from json config
	const String simName = simConfig["name"].get<std::string>();
	Real timeStep = simConfig["timestep"].get<double>();
	Real finalTime = simConfig["duration"].get<double>();

	//Switch to trigger fault at generator terminal
	Real SwitchOpen = 1e12;
	Real SwitchClosed = 0.1;

	Real timeStepPF = finalTime;
	Real finalTimePF = finalTime+timeStepPF;
	String simNamePF = simName + "_PF";
	Logger::setLogDir("logs/" + simNamePF);

	// Components
	auto n1PF = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2PF = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3PF = SimNode<Complex>::make("n3", PhaseType::Single);

	//Synchronous generator 1
	auto gen1PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::off);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen1PF->setParameters(ThreeBus.nomPower_G1, ThreeBus.nomPhPhVoltRMS_G1, ThreeBus.initActivePower_G1, ThreeBus.setPointVoltage_G1*ThreeBus.t1_ratio, PowerflowBusType::VD);
	gen1PF->setBaseVoltage(ThreeBus.Vnom);

	//Synchronous generator 2
	auto gen2PF = SP::Ph1::SynchronGenerator::make("Generator", Logger::Level::off);
	// setPointVoltage is defined as the voltage at the transfomer primary side and should be transformed to network side
	gen2PF->setParameters(ThreeBus.nomPower_G2, ThreeBus.nomPhPhVoltRMS_G2, ThreeBus.initActivePower_G2, ThreeBus.setPointVoltage_G2*ThreeBus.t2_ratio, PowerflowBusType::PV);
	gen2PF->setBaseVoltage(ThreeBus.Vnom);

	//use Shunt as Load for powerflow
	auto loadPF = SP::Ph1::Shunt::make("Load", Logger::Level::off);
	loadPF->setParameters(ThreeBus.activePower_L / std::pow(ThreeBus.Vnom, 2), - ThreeBus.reactivePower_L / std::pow(ThreeBus.Vnom, 2));
	loadPF->setBaseVoltage(ThreeBus.Vnom);
	
	//Line12
	auto line12PF = SP::Ph1::PiLine::make("PiLine12", Logger::Level::off);
	line12PF->setParameters(ThreeBus.lineResistance12, ThreeBus.lineInductance12, ThreeBus.lineCapacitance12, ThreeBus.lineConductance12);
	line12PF->setBaseVoltage(ThreeBus.Vnom);
	//Line13
	auto line13PF = SP::Ph1::PiLine::make("PiLine13", Logger::Level::off);
	line13PF->setParameters(ThreeBus.lineResistance13, ThreeBus.lineInductance13, ThreeBus.lineCapacitance13, ThreeBus.lineConductance13);
	line13PF->setBaseVoltage(ThreeBus.Vnom);
	//Line23
	auto line23PF = SP::Ph1::PiLine::make("PiLine23", Logger::Level::off);
	line23PF->setParameters(ThreeBus.lineResistance23, ThreeBus.lineInductance23, ThreeBus.lineCapacitance23, ThreeBus.lineConductance23);
	line23PF->setBaseVoltage(ThreeBus.Vnom);
	//Switch at n2
	auto faultN2PF = CPS::SP::Ph1::Switch::make("Br_fault_n2", Logger::Level::off);
	faultN2PF->setParameters(SwitchOpen, SwitchClosed);
	faultN2PF->open();
	//Switch at n3
	auto faultN3PF = CPS::SP::Ph1::Switch::make("Br_fault_n3", Logger::Level::off);
	faultN3PF->setParameters(SwitchOpen, SwitchClosed);
	faultN3PF->open();

	// Topology
	gen1PF->connect({ n1PF });
	gen2PF->connect({ n2PF });
	loadPF->connect({ n3PF });
	line12PF->connect({ n1PF, n2PF });
	line13PF->connect({ n1PF, n3PF });
	line23PF->connect({ n2PF, n3PF });
	faultN2PF->connect({SP::SimNode::GND , n2PF });
	faultN3PF->connect({SP::SimNode::GND , n3PF });
	auto systemPF = SystemTopology(60,
			SystemNodeList{n1PF, n2PF, n3PF},
			SystemComponentList{gen1PF, gen2PF, loadPF, line12PF, line13PF, line23PF, faultN2PF, faultN3PF});

	// Logging
	auto loggerPF = DataLogger::make(simNamePF);
	loggerPF->addAttribute("v_bus1", n1PF->attribute("v"));
	loggerPF->addAttribute("v_bus2", n2PF->attribute("v"));
	loggerPF->addAttribute("v_bus3", n3PF->attribute("v"));

	// Simulation
	Simulation simPF(simNamePF, Logger::Level::off);
	simPF.setSystem(systemPF);
	simPF.setTimeStep(timeStepPF);
	simPF.setFinalTime(finalTimePF);
	simPF.setDomain(Domain::SP);
	simPF.setSolverType(Solver::Type::NRP);
	simPF.doInitFromNodesAndTerminals(false);
	simPF.addLogger(loggerPF);
	simPF.run();

	// ----- Dynamic simulation ------
	String simNameDP = simName + "_DP";
	Logger::setLogDir("logs/"+simNameDP);
	
	// Nodes
	auto n1DP = SimNode<Complex>::make("n1", PhaseType::Single);
	auto n2DP = SimNode<Complex>::make("n2", PhaseType::Single);
	auto n3DP = SimNode<Complex>::make("n3", PhaseType::Single);

	// Components
	//Synchronous generator 1
	auto gen1DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen1", Logger::Level::off);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen1DP->setStandardParametersPU(ThreeBus.nomPower_G1, ThreeBus.nomPhPhVoltRMS_G1, ThreeBus.nomFreq_G1, ThreeBus.Xpd_G1*std::pow(ThreeBus.t1_ratio,2), ThreeBus.H_G1, ThreeBus.Rs_G1, ThreeBus.D_G1);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G1= gen1PF->getApparentPower();
	gen1DP->setInitialValues(initApparentPower_G1, ThreeBus.initMechPower_G1);

	//Synchronous generator 2
	auto gen2DP = DP::Ph1::SynchronGeneratorTrStab::make("SynGen2", Logger::Level::off);
	// Xpd is given in p.u of generator base at transfomer primary side and should be transformed to network side
	gen2DP->setStandardParametersPU(ThreeBus.nomPower_G2, ThreeBus.nomPhPhVoltRMS_G2, ThreeBus.nomFreq_G2, ThreeBus.Xpd_G2*std::pow(ThreeBus.t2_ratio,2), ThreeBus.H_G2, ThreeBus.Rs_G2, ThreeBus.D_G2);
	// Get actual active and reactive power of generator's Terminal from Powerflow solution
	Complex initApparentPower_G2= gen2PF->getApparentPower();
	gen2DP->setInitialValues(initApparentPower_G2, ThreeBus.initMechPower_G2);

	gen2DP->setModelFlags(true, true);
	gen2DP->setReferenceOmega(gen1DP->attribute<Real>("w_r"), gen1DP->attribute<Real>("delta_r"));
	
	///Load
	auto loadDP=DP::Ph1::RXLoad::make("Load", Logger::Level::off);
	loadDP->setParameters(ThreeBus.activePower_L, ThreeBus.reactivePower_L, ThreeBus.Vnom);

	//Line12
	auto line12DP = DP::Ph1::PiLine::make("PiLine12", Logger::Level::off);
	line12DP->setParameters(ThreeBus.lineResistance12, ThreeBus.lineInductance12, ThreeBus.lineCapacitance12, ThreeBus.lineConductance12);
	//Line13
	auto line13DP = DP::Ph1::PiLine::make("PiLine13", Logger::Level::off);
	line13DP->setParameters(ThreeBus.lineResistance13, ThreeBus.lineInductance13, ThreeBus.lineCapacitance13, ThreeBus.lineConductance13);
	//Line23
	auto line23DP = DP::Ph1::PiLine::make("PiLine23", Logger::Level::off);
	line23DP->setParameters(ThreeBus.lineResistance23, ThreeBus.lineInductance23, ThreeBus.lineCapacitance23, ThreeBus.lineConductance23);

	// Variable resistance switch at N2
	auto faultN2DP = std::make_shared<DP::Ph1::TriggeredSwitch>("Br_fault_n2", Logger::Level::off);
	faultN2DP->setParameters(SwitchOpen, SwitchClosed);
	faultN2DP->setInitParameters(timeStep);
	faultN2DP->open();

	// Variable resistance switch at N3
	auto faultN3DP = std::make_shared<DP::Ph1::TriggeredSwitch>("Br_fault_n3", Logger::Level::off);
	faultN3DP->setParameters(SwitchOpen, SwitchClosed);
	faultN3DP->setInitParameters(timeStep);
	faultN3DP->open();

	// Topology
	gen1DP->connect({ n1DP });
	gen2DP->connect({ n2DP });
	loadDP->connect({ n3DP });
	line12DP->connect({ n1DP, n2DP });
	line13DP->connect({ n1DP, n3DP });
	line23DP->connect({ n2DP, n3DP });
	faultN2DP->connect({DP::SimNode::GND , n2DP });
	faultN3DP->connect({DP::SimNode::GND , n3DP });

	auto systemDP = SystemTopology(60,
			SystemNodeList{n1DP, n2DP, n3DP},
			SystemComponentList{gen1DP, gen2DP, loadDP, line12DP, line13DP, line23DP, faultN2DP, faultN3DP});

	// Initialization of dynamic topology
	CIM::Reader reader(simNameDP, Logger::Level::off);
	reader.initDynamicSystemTopologyWithPowerflow(systemPF, systemDP);

	CPS::Logger::Log mSimulationLogCLI = Logger::get(simName, Logger::Level::off, Logger::Level::info);

	RealTimeSimulation simDP(simNameDP, CPS::Logger::Level::info);
	simDP.setSystem(systemDP);
	simDP.setTimeStep(timeStep);
	simDP.setFinalTime(finalTime);
	simDP.setDomain(Domain::DP);
	simDP.doSystemMatrixRecomputation(true);

	InterfaceShmem intf("/dpsim1-villas", "/villas-dpsim1", nullptr, false);
	simDP.addInterface(&intf,false);
	
	// Imported signals
	simDP.importIdObjAttr("Br_fault_n2","trigger_signal",0);
	simDP.importIdObjAttr("Br_fault_n2","closed_duration",1);
	simDP.importIdObjAttr("SynGen2","damping",2);
	simDP.importIdObjAttr("Br_fault_n3","trigger_signal",3);
	simDP.importIdObjAttr("Br_fault_n3","closed_duration",4);

	// Exported signals
	simDP.exportIdObjAttr("SynGen1","v_intf",0,CPS::AttributeBase::Modifier::mag);
	simDP.exportIdObjAttr("SynGen1","i_intf",1,CPS::AttributeBase::Modifier::mag);
	simDP.exportIdObjAttr("SynGen1","w_r",2);
	simDP.exportIdObjAttr("SynGen1","P_elec",3);
	simDP.exportIdObjAttr("SynGen2","v_intf",4,CPS::AttributeBase::Modifier::mag);
	simDP.exportIdObjAttr("SynGen2","i_intf",5,CPS::AttributeBase::Modifier::mag);
	simDP.exportIdObjAttr("SynGen2","w_r",6);
	simDP.exportIdObjAttr("SynGen2","P_elec",7);
	simDP.exportIdObjAttr("SynGen2","delta_r_rel",8);	
	
	// Further exported signals for debugging
	simDP.exportIdObjAttr("SynGen1","damping",9);
	simDP.exportIdObjAttr("SynGen2","damping",10);
	simDP.exportIdObjAttr("Br_fault_n2","is_closed",11);
	simDP.exportIdObjAttr("Br_fault_n2","trigger_signal",12);
	simDP.exportIdObjAttr("Br_fault_n2","closed_duration",13);
	simDP.exportIdObjAttr("Br_fault_n3","is_closed",14);
	simDP.exportIdObjAttr("Br_fault_n3","trigger_signal",15);
	simDP.exportIdObjAttr("Br_fault_n3","closed_duration",16);

	simDP.run();
	mSimulationLogCLI->info("overruns: {}", simDP.attribute<Int>("overruns")->getByValue());

	return 0;
}