#include "CFuzzySpeedController.h"

CFuzzySpeedController::CFuzzySpeedController(const char* name, const char* description){
	engine = new fl::Engine;
	engine->setName("SpeedController");
	engine->setDescription("");

	//distances in meters
	distanceError = new fl::InputVariable;
	distanceError->setName("distanceError");
	distanceError->setDescription("");
	distanceError->setEnabled(true);
	distanceError->setRange(0.0, std::numeric_limits<double>::infinity());
	distanceError->setLockValueInRange(false);
	distanceError->addTerm(new fl::Trapezoid("Zero", 0.0, 0.25, 0.50, 1.0));
	distanceError->addTerm(new fl::Triangle("Close", 0.5, 1.0, 2.0));
	distanceError->addTerm(new fl::Triangle("Far", 1.0, 2.0, 3.0));
	distanceError->addTerm(new fl::Triangle("VeryFar", 2.0, 3.0, std::numeric_limits<double>::infinity()));
	engine->addInputVariable(distanceError);

	//radians
	angleError = new fl::InputVariable;
	angleError->setName("angleError");
	angleError->setDescription("");
	angleError->setEnabled(true);
	angleError->setRange(-6.3, 6.3);
	angleError->setLockValueInRange(false);
	angleError->addTerm(new fl::Triangle("NFar", -6.3, -0.2, -0.1));
	angleError->addTerm(new fl::Triangle("NNear", -0.2, -0.15, 0.1));
	angleError->addTerm(new fl::Trapezoid("Zero", -0.15, -0.1, 0.1, 0.15));
	angleError->addTerm(new fl::Triangle("PNear", 0.1, 0.15, 0.2));
	angleError->addTerm(new fl::Triangle("PFar", 0.15, 0.2, 6.3));
	engine->addInputVariable(angleError);


	//linear speed in mm/s
	linearVelocity = new fl::OutputVariable;
	linearVelocity->setName("linearVelocity");
	linearVelocity->setDescription("");
	linearVelocity->setEnabled(true);
	linearVelocity->setRange(0, 0.3);
	linearVelocity->setLockValueInRange(false);
	linearVelocity->setAggregation(new fl::AlgebraicSum);
	linearVelocity->setDefuzzifier(new fl::Centroid(100));
	linearVelocity->setDefaultValue(0.0);
	linearVelocity->setLockPreviousValue(false);
	linearVelocity->addTerm(new fl::Triangle("Zero", 0.0, 0.05, 0.1));
	linearVelocity->addTerm(new fl::Triangle("SlowForward", 0.05, 0.1, 0.15));
	linearVelocity->addTerm(new fl::Triangle("Forward", 0.15, 0.2, 0.25));
	linearVelocity->addTerm(new fl::Triangle("FastForward", 0.2, 0.25, 0.3));
	engine->addOutputVariable(linearVelocity);
	
	/*angular speed in radians per second*/
	angularVelocity = new fl::OutputVariable;
	angularVelocity->setName("angularVelocity");
	angularVelocity->setDescription("");
	angularVelocity->setEnabled(true);
	angularVelocity->setRange(-0.3, 0.3);
	angularVelocity->setLockValueInRange(false);
	angularVelocity->setAggregation(new fl::AlgebraicSum);
	angularVelocity->setDefuzzifier(new fl::Centroid(100));
	angularVelocity->setDefaultValue(0.0);
	angularVelocity->setLockPreviousValue(false);
	angularVelocity->addTerm(new fl::Triangle("FastRight", -0.3, -0.25, -0.2));
	angularVelocity->addTerm(new fl::Triangle("SlowRight", -0.25, -0.2, -0.15));
	angularVelocity->addTerm(new fl::Trapezoid("Zero", -0.2, -0.15, 0.15, 0.2));
	angularVelocity->addTerm(new fl::Triangle("SlowLeft", 0.15, 0.2, 0.25));
	angularVelocity->addTerm(new fl::Triangle("FastLeft", 0.2, 0.25, 0.3));
	engine->addOutputVariable(angularVelocity);

	ruleBlock = new fl::RuleBlock;
	ruleBlock->setName("mamdani");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new fl::AlgebraicProduct);
	ruleBlock->setDisjunction(new fl::AlgebraicSum);
	ruleBlock->setImplication(new fl::AlgebraicProduct);
	ruleBlock->setActivation(new fl::General);

	ruleBlock->addRule(fl::Rule::parse("if angleError is NFar then linearVelocity is Zero and angularVelocity is FastRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is NNear then linearVelocity is Zero and angularVelocity is SlowRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is PNear then linearVelocity is Zero and angularVelocity is SlowLeft", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is PFar then linearVelocity is Zero and angularVelocity is FastLeft", engine));

	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is VeryFar then linearVelocity is FastForward and angularVelocity is Zero", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is Far then linearVelocity is Forward and angularVelocity is Zero", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is Close then linearVelocity is SlowForward and angularVelocity is Zero", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is Zero then linearVelocity is Zero and angularVelocity is Zero", engine));

	engine->addRuleBlock(ruleBlock);
	fl::fuzzylite::setDebugging(false);
	ROS_INFO("%s has been instantiated...", name);
}

CFuzzySpeedController::~CFuzzySpeedController(){
	delete engine;
}	

void CFuzzySpeedController::getTarget(double* distanceTarget, double* angleTarget){
	*distanceTarget = this->distanceTarget;
	*angleTarget = this->angleTarget;
}

void CFuzzySpeedController::setTarget(const double& distanceTarget, const double& angleTarget){
	this->distanceTarget = distanceTarget;
	this->angleTarget = angleTarget;
}

void CFuzzySpeedController::getSystemInput(const double& distance, const double& angle, double* linearVelocity, double* angularVelocity){

	this->distanceError->setValue(distance);
	this->angleError->setValue(angle);

	engine->process();

	*linearVelocity = this->linearVelocity->getValue();
	*angularVelocity = this->angularVelocity->getValue();
}