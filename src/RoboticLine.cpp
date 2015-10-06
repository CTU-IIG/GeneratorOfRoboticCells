/*
	This file is part of the GeneratorOfDatasets program.

	GeneratorOfDatasets is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	GeneratorOfDatasets is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with GeneratorOfDatasets. If not, see <http://www.gnu.org/licenses/>.
*/
#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <iterator>
#include <functional>
#include <stdexcept>
#include <random>
#include <unordered_set>
#include "RoboticLine.h"

using namespace std;

random_device rd;
default_random_engine generator(rd());

Movement::Movement(const uint32_t& from, const uint32_t& to, const ProjectParameters& par) : mFrom(from), mTo(to)	{
	uniform_real_distribution<double> durMin(par.minDurationOfMovement.from(), par.minDurationOfMovement.to());
	uniform_real_distribution<double> prolongation(par.prolongationOfMovement.from(), par.prolongationOfMovement.to());
	mMinDuration = durMin(generator);
	mMaxDuration = mMinDuration+prolongation(generator);

	for (int32_t degree = par.degreeOfCoefficients.from(), i = 0; degree <= par.degreeOfCoefficients.to(); ++degree, ++i)	{
		uniform_real_distribution<double> coeff(par.energyFunctionCoefficients[i].from(), par.energyFunctionCoefficients[i].to());
		mEnergyFunction.push_back({degree, coeff(generator)});
	}
}

LocationDependentPowerConsumption::LocationDependentPowerConsumption(RobotPowerMode* mode, const uint32_t& modeIdx, const ProjectParameters& par) : mMode(mode)	{
	uniform_real_distribution<double> inputPower(par.inputPowerOfModes[modeIdx].from(), par.inputPowerOfModes[modeIdx].to());
	mInputPower = inputPower(generator);
}

Location::Location(const uint32_t& point, const vector<RobotPowerMode*>& robotModes, const ProjectParameters& par) : mPoint(point) {
	for (uint32_t m = 0; m < robotModes.size(); ++m)	{
		if (par.inputPowerOfModes[m].from() < par.inputPowerOfModes[m].to())
			mLocationDependentPowerConsumption.emplace_back(robotModes[m], m, par);
	}
}

StaticActivity::StaticActivity(const uint32_t& aid, uint32_t& fromPoint, const ActivityType& type,
		const vector<RobotPowerMode*>& robotModes, const ProjectParameters& par) : Activity(aid, type)	{

	uniform_int_distribution<uint32_t> pt(par.numberOfPoints.from(), par.numberOfPoints.to());
	uniform_real_distribution<double> minDur(par.minDurationOfOperation.from(), par.minDurationOfOperation.to());
	uniform_real_distribution<double> prolongation(par.prolongationOfOperation.from(), par.prolongationOfOperation.to());

	mMinAbsDuration = minDur(generator);
	mMaxAbsDuration = mMinAbsDuration+prolongation(generator);
	uint32_t numberOfPoints = pt(generator);

	for (uint32_t p = fromPoint; p < fromPoint+numberOfPoints; ++p)	{
		Location *sp = new Location(p, robotModes, par);
		mLocations.push_back(sp);
	}

	double minPowerModeDelay = numeric_limits<double>::max();
	for (RobotPowerMode *m : robotModes)
		minPowerModeDelay = min(minPowerModeDelay, m->minimalDelay());

	if (mMaxAbsDuration < minPowerModeDelay)
		throw runtime_error("Location::Location(...): Instance is infeasible due to power-save mode delays!");

	mMinAbsDuration = max(mMinAbsDuration, minPowerModeDelay);
	fromPoint += numberOfPoints;
}

StaticActivity::~StaticActivity()	{
	for (Location *sp : mLocations)
		delete sp;
}

DynamicActivity::DynamicActivity(const uint32_t& aid, StaticActivity* fromActivity,
		StaticActivity* toActivity, const ProjectParameters& par) : Activity(aid, MOVEMENT) {

	vector<pair<uint32_t,uint32_t> > movements;
	size_t s1 = fromActivity->locations().size();
	size_t s2 = toActivity->locations().size();

	for (uint32_t i = 0; i < s1; ++i)
		for (uint32_t j = 0; j < s2; ++j)
			movements.emplace_back(i,j);

	shuffle(movements.begin(), movements.end(), generator);
	vector<bool> fromCovered(s1, false), toCovered(s2, false);

	for (const auto& c : movements)	{
		Movement *mv = new Movement(fromActivity->locations()[c.first]->point(), toActivity->locations()[c.second]->point(), par);
		mMinAbsDuration = min(mMinAbsDuration, mv->minDuration());
		mMaxAbsDuration = max(mMaxAbsDuration, mv->maxDuration());
		mMovements.push_back(mv);
		fromCovered[c.first] = true;
		toCovered[c.second] = true;

		if (count(fromCovered.cbegin(), fromCovered.cend(), false) + count(toCovered.cbegin(), toCovered.cend(), false) == 0)
			break;
	}

	fromActivity->addSuccessor(this);
	toActivity->addPredecessor(this);

	addSuccessor(toActivity);
	addPredecessor(fromActivity);
}

DynamicActivity::~DynamicActivity()	{
	for (Movement *mv : mMovements)
		delete mv;
}

Robot::Robot(uint32_t& fromActivityId, uint32_t& fromPointId, const uint32_t& numberOfInputs,
		const uint32_t& numberOfOutputs, const ProjectParameters& par) : mParameters(par) {

	uniform_int_distribution<uint32_t> psm(1u, (uint32_t) par.minDelayOfModes.size());

	uint32_t numberOfPowerSaveModes = psm(generator);
	for (uint32_t m = 0; m < numberOfPowerSaveModes; ++m)	{
		uniform_real_distribution<double> delay(par.minDelayOfModes[m].from(), par.minDelayOfModes[m].to());
		RobotPowerMode *rpwr = new RobotPowerMode(m, delay(generator));
		rpwr->powerSaveModeName("power mode "+numberToString(m));
		if (par.inputPowerOfModes[m].from() == par.inputPowerOfModes[m].to())
			rpwr->expectedInputPower(par.inputPowerOfModes[m].from());
		mRobotModes.push_back(rpwr);
	}

	// Create operations for the robot.
	vector<mCompositeBlock> operations;
	uint32_t numberOfOperations = min(numberOfInputs, numberOfOutputs);
	for (uint32_t op = 0; op+1 < numberOfOperations; ++op)
		operations.push_back(createSerialSequence(fromActivityId, fromPointId));

	int32_t diffDeg = numberOfOutputs-numberOfInputs;
	if (diffDeg > 0)
		operations.push_back(createDisassemblyBlock(fromActivityId, fromPointId, abs(diffDeg)+1));
	else if (diffDeg < 0)
		operations.push_back(createAssemblyBlock(fromActivityId, fromPointId, abs(diffDeg)+1));
	else
		operations.push_back(createSerialSequence(fromActivityId, fromPointId));


	mCompositeBlock operationsBlock = mergeBlocks(operations);
	for (uint32_t opOut = 0; opOut < operations.size(); ++opOut)	{
		for (uint32_t opIn = 0; opIn < operations.size(); ++opIn)	{
			if (opOut != opIn)
				connect(fromActivityId, operationsBlock, operations[opOut].out, operations[opIn].in);
		}
	}

	StaticActivity *waitToNextCycle = new StaticActivity(fromActivityId++, fromPointId, WAIT, mRobotModes, mParameters);
	connect(fromActivityId, operationsBlock, operationsBlock.out, {waitToNextCycle});
	connect(fromActivityId, operationsBlock, {waitToNextCycle}, operationsBlock.in);

	mActivities.insert(mActivities.end(), operationsBlock.in.cbegin(), operationsBlock.in.cend());
	mActivities.insert(mActivities.end(), operationsBlock.w.cbegin(), operationsBlock.w.cend());
	mActivities.insert(mActivities.end(), operationsBlock.out.cbegin(), operationsBlock.out.cend());
	mActivities.push_back(waitToNextCycle);
	mActivities.insert(mActivities.end(), operationsBlock.mv.cbegin(), operationsBlock.mv.cend());

	sort(mActivities.begin(), mActivities.end(), [](const Activity* a, const Activity* b) { return a->aid() < b->aid() ? true : false; });
}

double Robot::lowerBoundOfCycleTime() const	{
	double sumOfDurations = 0.0;
	size_t numberOfStatAct = 0;
	vector<double> durationOfMovements;
	for (const Activity* a : mActivities)	{
		if (a->type() != MOVEMENT)	{
			sumOfDurations += a->minAbsDuration();
			++numberOfStatAct;
		} else	{
			durationOfMovements.push_back(a->minAbsDuration());
		}
	}

	sort(durationOfMovements.begin(), durationOfMovements.end());
	sumOfDurations += accumulate(durationOfMovements.begin(), durationOfMovements.begin()
					+ min(numberOfStatAct, durationOfMovements.size()), 0.0);

	return sumOfDurations;
}

Robot::~Robot()	{
	for (RobotPowerMode *m : mRobotModes)
		delete m;
	for (Activity* a : mActivities)
		delete a;
}

Robot::mCompositeBlock Robot::createSerialSequence(uint32_t& fromActivityId, uint32_t& fromPointId) const	{

	mCompositeBlock block = createBlockOfOperations(fromActivityId, fromPointId, IN, OUT, {1, 1},
			{mParameters.sequenceLength.from()+2, mParameters.sequenceLength.to()+2});

	return block;
}


Robot::mCompositeBlock Robot::createAssemblyBlock(uint32_t& fromActivityId, uint32_t& fromPointId, const uint32_t& numberOfInputs) const {
	return createComplexBlockHelper(fromActivityId, fromPointId, numberOfInputs, "assembly");
}

Robot::mCompositeBlock Robot::createDisassemblyBlock(uint32_t& fromActivityId, uint32_t& fromPointId, const uint32_t& numberOfOutputs) const	{
	return createComplexBlockHelper(fromActivityId, fromPointId, numberOfOutputs, "disassembly");
}

Robot::mCompositeBlock Robot::createComplexBlockHelper(uint32_t& fromActivityId, uint32_t& fromPointId, const uint32_t& numberOfSeq, const string& op) const	{

	mCompositeBlock block, block1, block2, block3;

	if (op == "assembly")	{
		block1 = createBlockOfOperations(fromActivityId, fromPointId, IN, INNER, {numberOfSeq, numberOfSeq}, {2,2});
		block3 = createBlockOfOperations(fromActivityId, fromPointId, OUT, OUT, {1, 1}, {1, 1});
	} else if (op == "disassembly")	{
		block1 = createBlockOfOperations(fromActivityId, fromPointId, IN, IN, {1, 1}, {1, 1});
		block3 = createBlockOfOperations(fromActivityId, fromPointId, INNER, OUT, {numberOfSeq, numberOfSeq}, {2, 2});
	} else {
		throw runtime_error("Robot::mCompositeBlock Robot::createComplexBlockHelper(...): Requested operation is not supported!");
	}

	block2 = createBlockOfOperations(fromActivityId, fromPointId, INNER, INNER, mParameters.numberOfSequences, mParameters.sequenceLength);

	block = mergeBlocks({block1, block2, block3});

	connect(fromActivityId, block, block1.con2, block2.con1);
	connect(fromActivityId, block, block2.con2, block3.con1);

	return block;
}

Robot::mCompositeBlock Robot::createBlockOfOperations(uint32_t& fromActivityId, uint32_t& fromPointId, const ActivityType& t1, const ActivityType& t2,
		const Interval<uint32_t>& numSeq, const Interval<uint32_t>& seqLength) const	{

	uniform_int_distribution<uint32_t> sl(seqLength.from(), seqLength.to());
	uniform_int_distribution<uint32_t> ns(numSeq.from(), numSeq.to());

	mCompositeBlock blockActivities;
	uint32_t numberOfSequences = ns(generator);

	function<void(StaticActivity*)> storeActivity = [&blockActivities](StaticActivity *a) {
		switch (a->type())	{
			case IN:
				blockActivities.in.push_back(a);
				break;
			case INNER:
				blockActivities.w.push_back(a);
				break;
			default:
				blockActivities.out.push_back(a);
		}
	};


	for (uint32_t i = 0; i < numberOfSequences; ++i)	{

		uint32_t sequenceLength = sl(generator);

		StaticActivity *op1 = new StaticActivity(fromActivityId++, fromPointId, t1, mRobotModes, mParameters);
		blockActivities.con1.push_back(op1);
		storeActivity(op1);

		StaticActivity *op2 = nullptr;
		for (uint32_t l = 0; l+2 < sequenceLength; ++l)	{
			op2 = new StaticActivity(fromActivityId++, fromPointId, INNER, mRobotModes, mParameters);
			DynamicActivity *op2op = new DynamicActivity(fromActivityId++, op1, op2, mParameters);
			blockActivities.w.push_back(op2);
			blockActivities.mv.push_back(op2op);
			op1 = op2;
		}

		if (sequenceLength == 1)	{
			if (t1 == t2)	{
				op2 = op1;
			} else {
				throw runtime_error("Robot::mCompositeBlock Robot::createBlockOfOperations(...): "
						"Sequence of unit length must have the same types of the first and last activity");
			}
		} else {
			op2 = new StaticActivity(fromActivityId++, fromPointId, t2, mRobotModes, mParameters);
			DynamicActivity *op2op = new DynamicActivity(fromActivityId++, op1, op2, mParameters);
			blockActivities.mv.push_back(op2op);
			storeActivity(op2);
		}

		blockActivities.con2.push_back(op2);
	}

	connect(fromActivityId, blockActivities, blockActivities.con2, blockActivities.con1, false);

	return blockActivities;
}

void Robot::connect(uint32_t& fromActivityId, mCompositeBlock& b, const vector<StaticActivity*>& from, const vector<StaticActivity*>& to, bool fullyConnected) const	{
	for (uint32_t i1 = 0; i1 < from.size(); ++i1)	{
		for (uint32_t i2 = 0; i2 < to.size(); ++i2)	{
			if (fullyConnected || i1 != i2)	{
				DynamicActivity *mv = new DynamicActivity(fromActivityId++, from[i1], to[i2], mParameters);
				b.mv.push_back(mv);
			}
		}
	}
}

Robot::mCompositeBlock Robot::mergeBlocks(const vector<mCompositeBlock>& blocks) const	{
	mCompositeBlock ret;
	for (const mCompositeBlock& b : blocks)	{
		copy(b.in.cbegin(), b.in.cend(), back_inserter(ret.in));
		copy(b.w.cbegin(), b.w.cend(), back_inserter(ret.w));
		copy(b.out.cbegin(), b.out.cend(), back_inserter(ret.out));
		copy(b.mv.cbegin(), b.mv.cend(), back_inserter(ret.mv));
	}

	return ret;
}

bool Pair::operator==(const Pair& p) const	{
	if (mActivity1 == p.mActivity1 && mActivity2 == p.mActivity2 && mMode1 == p.mMode1 && mMode2 == p.mMode2)
		return true;
	else
		return false;
}

InterRobotOperation::InterRobotOperation(uint32_t oid, Activity* out, Activity* in, const ProjectParameters& par) : mOid(oid)	{

	uniform_real_distribution<double> handover(0.0, 100.0);

	if (handover(generator) < par.percentageOfTableHandover)	{
		// bench
		for (Activity *succ : out->successors())
			mTimeLags.emplace_back(succ, in, 0, 0);
		for (Activity *succ : in->successors())
			mTimeLags.emplace_back(succ, out, 0, 1);

		name("Inter-robot operation "+numberToString(oid)+" - bench handover");
	} else {
		// gripper-to-gripper
		mTimeLags.emplace_back(out, in, 0, 0);
		mTimeLags.emplace_back(in, out, 0, 0);
		for (Activity *inSucc : in->successors())	{
			for (Activity *outSucc : out->successors())	{
				mTimeLags.emplace_back(inSucc, outSucc, 0, 0);
				mTimeLags.emplace_back(outSucc, inSucc, 0, 0);
			}
		}

		name("Inter-robot operation "+numberToString(oid)+" - gripper-to-gripper handover");
	}

	generateSpatialCompatibilityPairs(out, in);
}

void InterRobotOperation::generateSpatialCompatibilityPairs(Activity* a1, Activity* a2) {

	StaticActivity *sa1 = dynamic_cast<StaticActivity*>(a1);
	StaticActivity *sa2 = dynamic_cast<StaticActivity*>(a2);
	if (sa1 == nullptr || sa2 == nullptr)
		throw invalid_argument("void InterRobotOperation::generateSpatialCompatibilityPairs(...): Input activities must be static ones!");

	vector<Location*> loc1 = sa1->locations(), loc2 = sa2->locations();
	if (loc1.empty() || loc2.empty())
		throw runtime_error("void InterRobotOperation::generateSpatialCompatibilityPairs(...): Static activity without interior locations!?");


	vector<pair<uint32_t, uint32_t> > possibleLocPairs;
	for (uint32_t i = 0; i < loc1.size(); ++i)
		for (uint32_t j = 0; j < loc2.size(); ++j)
			possibleLocPairs.emplace_back(i,j);

	vector<bool> usedLoc1(loc1.size(), false), usedLoc2(loc2.size(), false);
	shuffle(possibleLocPairs.begin(), possibleLocPairs.end(), generator);

	for (auto it = possibleLocPairs.cbegin(); it != possibleLocPairs.cend(); ++it)	{
		usedLoc1[it->first] = usedLoc2[it->second] = true;
		mSpatialCompatibility.emplace_back(a1, it->first, a2, it->second);

		if (count(usedLoc1.begin(), usedLoc1.end(), false) + count(usedLoc2.begin(), usedLoc2.end(), false) == 0)
			break;
	}
}


RoboticLine::RoboticLine(const ProjectParameters& par)	{

	uint32_t nr = par.numberOfRobots;
	vector<pair<uint32_t, uint32_t> > possibleInterconnections;
	uniform_real_distribution<double> graphDeg(par.minimalVertexDegree.from(), par.minimalVertexDegree.to());

	for (uint32_t r1 = 0; r1 < nr; ++r1)	{
		for (uint32_t r2 = 0; r2 < nr; ++r2)	{
			if (r1 != r2 && r2 != 0 && r1+1 != nr)
				possibleInterconnections.emplace_back(r1, r2);
		}
	}

	uint32_t sumOfDegrees = 0;
	shuffle(possibleInterconnections.begin(), possibleInterconnections.end(), generator);
	double minimalAverageDegree = graphDeg(generator), averageDegree = 0.0, inf = numeric_limits<double>::infinity();
	vector<vector<double> > distMatrix(nr, vector<double>(nr, inf));

	for (const auto& interconnection : possibleInterconnections)	{

		auto distMatrixCopy = distMatrix;
		uint32_t r1 = interconnection.first, r2 = interconnection.second;

		distMatrix[r1][r2] = 1.0;

		for (uint32_t k : {r1, r2})	{
			for (uint32_t i = 0; i < nr; ++i)	{
				for (uint32_t j = 0; j < nr; ++j)	{
					if (distMatrix[i][k]+distMatrix[k][j] < distMatrix[i][j])
						distMatrix[i][j] = distMatrix[i][k]+distMatrix[k][j];
				}
			}
		}

		bool acyclic = true;
		for (uint32_t i = 0; i < nr; ++i)	{
			if (distMatrix[i][i] < inf)	{
				acyclic = false;
				break;
			}
		}

		if (!acyclic)	{
			// backtrack: only acyclic graphs are allowed
			distMatrix = distMatrixCopy;
			continue;
		}

		bool connectedEnough = true;
		for (uint32_t r = 0; r < nr && connectedEnough; ++r)	{
			bool robotConnected = false;
			for (uint32_t rb = 0; rb < nr; ++rb)	{
				if (distMatrix[rb][r] < inf || r == 0)	{
					robotConnected = true;
					break;
				}
			}

			robotConnected = (distMatrix[r][nr-1] < inf || r+1 == nr) ? robotConnected : false;
			connectedEnough = (connectedEnough && robotConnected) ? true : false;
		}

		sumOfDegrees += 2;
		averageDegree = ((double) sumOfDegrees)/((double) nr);

		if (connectedEnough == true && averageDegree >= minimalAverageDegree)
			break;
	}

	if (averageDegree < minimalAverageDegree)	{
		mWarnings.push_back("The average degree of nodes (desired "+numberToString(minimalAverageDegree)
				+", obtained "+numberToString(averageDegree)+") cannot be reached.");
	}

	uint32_t fromActivityId = 0, fromPointId = 0;
	for (uint32_t r1 = 0; r1 < nr; ++r1)	{
		uint32_t numberOfInputs = 0, numberOfOutputs = 0;
		for (uint32_t r2 = 0; r2 < nr; ++r2)	{
			if (r1 != r2 && distMatrix[r1][r2] < inf)
				++numberOfOutputs;
			if (r1 != r2 && distMatrix[r2][r1] < inf)
				++numberOfInputs;
		}

		if (r1 == 0)
			++numberOfInputs;
		if (r1+1 == par.numberOfRobots)
			++numberOfOutputs;

		if (numberOfInputs == 0 || numberOfOutputs == 0)
			throw runtime_error("RoboticLine::RoboticLine(...): Each robot has to have at least one input and one output activity!");

		mRobots.push_back(new Robot(fromActivityId, fromPointId, numberOfInputs, numberOfOutputs, par));
	}

	vector<vector<Activity*> > robotIn, robotOut;
	for (uint32_t r = 0; r < nr; ++r)	{
		vector<Activity*> inActivities, outActivities;
		vector<Activity*> activities = mRobots[r]->activities();
		copy_if(activities.cbegin(), activities.cend(), back_inserter(inActivities),
				[](Activity* const& a) { return (a->type() == IN ? true : false); });
		copy_if(activities.cbegin(), activities.cend(), back_inserter(outActivities),
				[](Activity* const& a) { return (a->type() == OUT ? true : false); });
		shuffle(inActivities.begin(), inActivities.end(), generator);
		shuffle(outActivities.begin(), outActivities.end(), generator);
		robotIn.push_back(inActivities); robotOut.push_back(outActivities);
	}

	uint32_t opId = 0;
	for (uint32_t r1 = 0; r1 < nr; ++r1)	{
		for (uint32_t r2 = 0; r2 < nr; ++r2)	{
			if (r1 != r2 && distMatrix[r1][r2] < inf)	{
				vector<Activity*>& out = robotOut[r1], & in = robotIn[r2];
				if (!out.empty() && !in.empty())  {
					mInterRobotOperations.emplace_back(opId++, out.back(), in.back(), par);
					out.pop_back(); in.pop_back();
				} else {
					throw runtime_error("RoboticLine::RoboticLine(...): Not enough IN and OUT inter-robot activities!");
				}
			}
		}
	}

	generateCollisions(par);
	calculateProductionCycleTime(par);
}

void RoboticLine::generateCollisions(const ProjectParameters& par)	{

	unordered_set<Pair> addedCollisions;
	uint32_t numberOfGeneratedCollisions = 0, iter = 0;
	uniform_int_distribution<uint32_t> robotId(0, par.numberOfRobots-1);
	uniform_int_distribution<uint32_t> numCol(par.numberOfCollisions.from(), par.numberOfCollisions.to());

	const uint32_t desiredNumberOfCollisions = numCol(generator);
	const uint32_t maxNumberOfIterations = 10*desiredNumberOfCollisions;

	while (numberOfGeneratedCollisions < desiredNumberOfCollisions)	{

		uint32_t robotId1 = robotId(generator), robotId2 = robotId(generator);

		if (robotId1 != robotId2)	{
			uniform_int_distribution<uint32_t> a1(0, ((int32_t) mRobots[robotId1]->activities().size())-1);
			uniform_int_distribution<uint32_t> a2(0, ((int32_t) mRobots[robotId2]->activities().size())-1);

			Activity *activity1 = mRobots[robotId1]->activities()[a1(generator)];
			Activity *activity2 = mRobots[robotId2]->activities()[a2(generator)];

			uniform_int_distribution<uint32_t> modeIdx1(0, ((int32_t) activity1->numberOfModes())-1);
			uniform_int_distribution<uint32_t> modeIdx2(0, ((int32_t) activity2->numberOfModes())-1);

			Pair collision = {activity1, modeIdx1(generator), activity2, modeIdx2(generator)};

			if (addedCollisions.count(collision) == 0)	{
				mCollisions.push_back(collision);
				addedCollisions.insert(collision);
				++numberOfGeneratedCollisions;
			}
		}


		if (iter > maxNumberOfIterations)	{
			mWarnings.push_back("Cannot generate the requested number of collisions!");
			break;
		}

		++iter;
	}
}

void RoboticLine::calculateProductionCycleTime(const ProjectParameters& par) {
	double productionCycleTime = 0;
	uniform_real_distribution<double> dilFactor(par.dilatationFactor.from(), par.dilatationFactor.to());

	for (Robot *r : mRobots)
		productionCycleTime = max(productionCycleTime, r->lowerBoundOfCycleTime());

	mProductionCycleTime = dilFactor(generator)*productionCycleTime;
}

RoboticLine::~RoboticLine()	{
	for (Robot *r : mRobots)
		delete r;
}

