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
#ifndef HLIDAC_PES_ROBOTIC_LINE_H
#define HLIDAC_PES_ROBOTIC_LINE_H

/*!
 * \file RoboticLine.h
 * \author Libor Bukata
 * \brief The file contains various classes devoted to abstract representation of the robotic cell.
 */

#include <iostream>
#include <stdint.h>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#include "ProjectParameters.h"

/*!
 * \brief It represents the part of energy functions, i.e. \f$\mathrm{coeff}*d^{\mathrm{degree}}\f$ where \f$d\f$ is the duration of the movement.
 */
struct Monomial {
	int32_t degree;
	double coeff;
};

/*!
 * The enum defines the constants for different types of activities.
 * For robot operations (see StaticActivity class), it is classified
 * with respect to the workpiece/weldment flow from the robot perspective,
 * i.e. IN - input, OUT - output, INNER - some robotic work, WAIT - waiting for work (dummy operation).
 * For robot movements (see DynamicActivity class), there is only one type, i.e. MOVEMENT.
 */
enum ActivityType {
	IN, OUT, INNER, WAIT, MOVEMENT,
};

/*!
 * \param number The value to be converted to string.
 * \tparam T Numeric type.
 * \return The value converted to string.
 * \brief It converts the number to string.
 * \note As C++11 'to_string' function is not declared for Cygwin g++ compiler it is necessary to workaround the bug.
 */
template <class T>
std::string numberToString(const T& number)	{
	std::ostringstream ss;
	ss<<number;
	return ss.str();
}

/*!
 * \brief The class represents the robot movement between two coordinates.
 */
class Movement {
	public:
		/*!
		 * \param from Identification of the start coordinate, see Location::mPoint.
		 * \param to Identification of the end coordinate, see Location::mPoint.
		 * \param par It contains the parameters for the movement generation.
		 * \brief The constructor creates the movement according to the desired properties given in the project configuration file.
		 */
		Movement(const uint32_t& from, const uint32_t& to, const ProjectParameters& par);

		uint32_t from() const {	return mFrom; }
		uint32_t to() const { return mTo; }
		double minDuration() const { return mMinDuration; }
		double maxDuration() const { return mMaxDuration; }
		std::vector<Monomial> energyFunction() const { return mEnergyFunction; }
	private:
		//! Identification of the start coordinate of the movement.
		uint32_t mFrom;
		//! Identification of the end coordinate of the movement.
		uint32_t mTo;
		//! The minimal possible duration of the robot movement.
		double mMinDuration;
		//! The maximal duration of the movement.
		double mMaxDuration;
		/*!
		 * Energy function, the relation between the duration and required energy for the movement,
		 * is modelled as a sum of monomials stored in mEnergyFunction vector.
		 */
		std::vector<Monomial> mEnergyFunction;
};

/*!
 * The class represents the power saving mode of the robot, which can be applied if the robot is in a stationary position.
 * For example, the stationary robot can usually be held by electric motors (dummy power saving mode) or brakes.
 * However, even deeper power saving modes such as bus-power-off and hibernate are possible for e.g. KUKA robots.
 * \brief It represents the power saving mode of the robot.
 */
class RobotPowerMode {
	public:
		/*!
		 * \param pid Identification of the power saving mode.
		 * \param delay The power saving mode is possible to apply if the robot is at least delay seconds in a stationary position.
		 * \brief It constructs the robot's power saving mode.
		 * \note If mExpectedInputPower is equal to -1, then the input power is dependent on the robot configuration, i.e. position.
		 */
		RobotPowerMode(const uint32_t& pid, const double& delay) : mPid(pid), mMinimalDelay(delay), mExpectedInputPower(-1.0) { }

		uint32_t pid() const { return mPid; }
		void pid(const uint32_t& pid) { mPid = pid; }
		void powerSaveModeName(const std::string& name) { mName = name; }
		std::string powerSaveModeName() const { return mName; }
		double minimalDelay() const { return mMinimalDelay; }
		double expectedInputPower() const { return mExpectedInputPower; }
		void expectedInputPower(const double& expectedInputPower) { mExpectedInputPower = expectedInputPower; }
	private:
		//! Identification of the robot power saving mode, unique for the robot.
		uint32_t mPid;
		//! The name of the power saving mode, e.g. "motors", "brakes", etc.
		std::string mName;
		//! The minimal time in seconds which is required for the stationary robot to apply this power saving mode.
		double mMinimalDelay;
		//! Expected input power of the robot if it is not dependent on the robot configuration (joint values), otherwise -1.
		double mExpectedInputPower;
};

/*!
 * Robot input power for some power saving modes is also dependent on the robot configuration, for example,
 * if the robot is held by motors then the power consumption is different for the robot being stretched out or robot at home position
 * that is usually much more energy effective. This class can be perceived as an extension to RobotPowerMode class as it adds
 * the information about input power for a particular robot configuration, i.e. location (see also the Location class).
 * \brief The class specifies input power of the robot for a particular robot configuration.
 */
class LocationDependentPowerConsumption {
	public:
		/*!
		 * \param mode The pointer to the related power saving mode of the robot.
		 * \param modeIdx The index addressing the interval of the desired input power.
		 * \param par It contains the intervals of input power for various robot power saving modes.
		 * \brief It constructs the object encapsulating the information about the input power for the given robot configuration.
		 */
		LocationDependentPowerConsumption(RobotPowerMode* mode, const uint32_t& modeIdx, const ProjectParameters& par);

		RobotPowerMode* robotModeRef() const { return mMode; }
		double inputPower() const { return mInputPower; }
	private:
		//! A pointer to the related power saving mode of the robot.
		RobotPowerMode *mMode;
		//! Input power of the robot for a given robot configuration, see also the Location class.
		double mInputPower;
};

/*!
 * The instance of the class corresponds to a robot configuration, in which the robot can perform an operation on the workpiece/weldment.
 * Even though the class is called Location, it represents not only the absolute coordinates of the operation but also orientation of the gun/gripper of the robot.
 * \brief It represents the location of robotic work.
 */
class Location {
	public:
		/*!
		 * \param point Identification of the robot configuration for a given location and orientation.
		 * \param robotModes Power saving modes of the robot from which the location is reachable.
		 * \param par It comprises of the desired parameters for the generation of robotic cells.
		 * \brief It constructs the object representing a location of an operation from the robot point of view.
		 */
		Location(const uint32_t& point, const std::vector<RobotPowerMode*>& robotModes, const ProjectParameters& par);

		uint32_t point() const { return mPoint; }
		std::vector<LocationDependentPowerConsumption> locationDependentPowerConsumption() const {
			return mLocationDependentPowerConsumption;
		}
	private:
		//! Identification of the robot configuration for the given location and operation.
		uint32_t mPoint;
		//! It defines the power consumption for the robot power saving modes that input power is dependent on the robot configuration.
		std::vector<LocationDependentPowerConsumption> mLocationDependentPowerConsumption;
};

/*!
 * The abstract class incorporating the common properties of both types of the activity, i.e. operations and movements.
 * Even though the robot operations, e.g assembling and welding, are considered as stationary, which is not true in nature,
 * it is no problem to decompose these operations further into more sub-operations and sub-movements to achieve higher model accuracy.
 * \brief The base class incorporating common properties for robot operations and movements.
 */
class Activity {
	public:
		/*!
		 * \param aid Identification of the activity.
		 * \param type The type of activity, see \ref ActivityType enum.
		 * \brief The base constructor called from the derived classes for initialization of attributes.
		 */
		Activity(const uint32_t& aid, const ActivityType& type) : mAid(aid), mType(type),
				mMinAbsDuration(std::numeric_limits<double>::max()), mMaxAbsDuration(std::numeric_limits<double>::min()) { }

		uint32_t aid() const { return mAid; }
		ActivityType type() const { return mType; }
		void successors(const std::vector<Activity*>& successors) { mSuccessors = successors; }
		void addSuccessor(Activity* successor) { mSuccessors.push_back(successor); }
		std::vector<Activity*> successors() const { return mSuccessors; }
		void predecessors(const std::vector<Activity*>& predecessors) { mPredecessors = predecessors; }
		void addPredecessor(Activity* predecessor) { mPredecessors.push_back(predecessor); }
		std::vector<Activity*> predecessors() const { return mPredecessors; }
		void minAbsDuration(const double& minAbsDuration) { mMinAbsDuration = minAbsDuration; }
		double minAbsDuration() const { return mMinAbsDuration; }
		void maxAbsDuration(const double& maxAbsDuration) { mMaxAbsDuration = maxAbsDuration; }
		double maxAbsDuration() const { return mMaxAbsDuration; }
		virtual size_t numberOfModes() const = 0;

		virtual ~Activity() { }
	private:
		Activity(const Activity&) = delete;
		Activity& operator=(const Activity&) = delete;
	protected:
		//! Identification of the activity.
		uint32_t mAid;
		//! The type of the activity, see ActivityType enum.
		ActivityType mType;
		//! The successors of the activity.
		std::vector<Activity*> mSuccessors;
		//! The predecessors of the activity.
		std::vector<Activity*> mPredecessors;
		//! The minimal absolute duration of the activity to remain feasible.
		double mMinAbsDuration;
		//! The maximal absolute duration of the activity to remain feasible.
		double mMaxAbsDuration;
};

/*!
 * The static activity represents a collection of locations, i.e. robot configurations, in which some operation such as
 * e.g. welding, cutting, and assembling can be performed. In the final solution, only one location is selected for each robot operation.
 * \brief Collection of locations in which a robot operation can be performed.
 */
class StaticActivity : public Activity {
	public:
		/*!
		 * \param aid Unique identification of the activity.
		 * \param fromPoint Initial unique identification of generated locations, i.e. robot configurations.
		 * \param type Type of the static activity, i.e. IN, INNER, WAIT, or OUT.
		 * \param robotModes The power saving modes of the robot for which this activity was created.
		 * \param par The desired properties of instances used for generation.
		 * \brief It generates the static activity, i.e. robot operation, according to the configuration.
		 */
		StaticActivity(const uint32_t& aid, uint32_t& fromPoint, const ActivityType& type,
			       	const std::vector<RobotPowerMode*>& robotModes, const ProjectParameters& par);

		std::vector<Location*> locations() const { return mLocations; }
		virtual size_t numberOfModes() const override { return mLocations.size(); }

		//! It frees the memory occupied by dynamically allocated locations.
		virtual ~StaticActivity() override;
	private:
		//! The collection of possible robot configurations, i.e. locations, for this operation.
		std::vector<Location*> mLocations;
};

/*!
 * The dynamic activity represents a collection of possible movements between two static activities.
 * In the final solution, at most one movement can be selected for each dynamic activity due to the alternatives,
 * i.e. the possibility to select from multiple process plans - activity orders.
 * \brief Collection of movements between two static activities.
 */
class DynamicActivity: public Activity {
	public:
		/*!
		 * \param aid Unique identification of the activity.
		 * \param fromActivity The static activity from which the movements are leaving.
		 * \param toActivity The static activity to which the movements are entering.
		 * \param par The parameters used for the generation.
		 * \brief It generates the dynamic activity with respect to the desired properties.
		 */
		DynamicActivity(const uint32_t& aid, StaticActivity* fromActivity,
				StaticActivity* toActivity, const ProjectParameters& par);

		std::vector<Movement*> movements() const { return mMovements; }
		virtual size_t numberOfModes() const override { return mMovements.size(); }

		//! It frees the memory occupied by dynamically allocated movements.
		virtual ~DynamicActivity() override;
	private:
		//! The collection of movements between two static activities.
		std::vector<Movement*> mMovements;
};

/*!
 * The instance of this class corresponds to a robot with its operations and movements.
 * The order of activities is incorporated in activities' successors and predecessors.
 * \brief Instance of the class includes all the data structures and methods related to a robot.
 */
class Robot {
	private:
		/*!
		 * It integrates activities with their interconnections into an easy-to-connect block,
		 * which can be e.g. cutting block, assembling block, welding block, etc.
		 * The structure contains the various types of activities:
		 * in ~ IN, w ~ INNER, out ~ OUT, mv ~ MOVEMENT, con1 ~ IN*, con2 ~ OUT*.
		 * Activities in con1 and con2 are inputs and outputs of the block respectively.
		 * \brief The structure containing the activities in a block.
		 */
		struct mCompositeBlock {
			std::vector<StaticActivity*> in, w, out;
			std::vector<DynamicActivity*> mv;
			std::vector<StaticActivity*> con1, con2;
		};

	public:
		/*!
		 * \param fromActivityId An initial activity numbering, i.e. unique identification.
		 * \param fromPointId An initial unique identification of locations, i.e. robot configurations.
		 * \param numberOfInputs The number of input static activities for taking the workpiece/weldment from other robot or shared bench.
		 * \param numberOfOutputs The number of output static activities for passing the workpiece/weldment to other robot directly or using shared bench.
		 * \param par Parameters required for generation of the robot.
		 * \brief It constructs the robot according to the desired parameters.
		 */
		Robot(uint32_t& fromActivityId, uint32_t& fromPointId, const uint32_t& numberOfInputs, const uint32_t& numberOfOutputs, const ProjectParameters& par);

		std::vector<Activity*> activities() const { return mActivities; }
		std::vector<RobotPowerMode*> robotModes() const { return mRobotModes; }

		/*!
		 * \return It returns the lower estimation of production cycle time for this robot.
		 * \brief The method calculates an estimation of cycle time.
		 */
		double lowerBoundOfCycleTime() const;

		//! It frees the memory occupied by dynamically allocated activities and power saving modes.
		~Robot();
	private:

		Robot(const Robot&) = delete;
		Robot& operator=(const Robot&) = delete;


		/*!
		 * \param fromActivityId, fromPointId It is a reference to an initial activity/location identification respectively.
		 * \return A block of a fixed sequence of operations.
		 * \brief The method creates a block of a fixed sequence of operations, e.g. welding, cutting, without alternatives and updates the identification values.
		 * \see mCompositeBlock, createBlockOfOperations
		 */
		mCompositeBlock createSerialSequence(uint32_t& fromActivityId, uint32_t& fromPointId) const;
		/*!
		 * \param fromActivityId, fromPointId It is a reference to an initial activity/location identification respectively.
		 * \param numberOfInputs The number of input parts to be assembled or welded.
		 * \return An assembling/welding block with alternatives.
		 * \brief It creates a block composed of gathering required parts to a bench and their assembly/welding to one workpiece
		 * \note Do not be confused by the name of the method, it can be also used for a creation of a welding block.
		 * \see mCompositeBlock, createComplexBlockHelper
		 */
		mCompositeBlock createAssemblyBlock(uint32_t& fromActivityId, uint32_t& fromPointId, const uint32_t& numberOfInputs) const;
		/*!
		 * \param fromActivityId, fromPointId It is a reference to an initial activity/location identification respectively.
		 * \param numberOfOutputs The number of parts to which the input workpiece is disassembled/cut.
		 * \return An disassembling/cutting block with alternatives.
		 * \brief It creates a block composed of disassembling/cutting the workpiece and taking its parts to other robots or benches.
		 * \note Do not be confused by the name of the method, it can be also used for a creation of a cutting block.
		 * \see mCompositeBlock, createComplexBlockHelper
		 */
		mCompositeBlock createDisassemblyBlock(uint32_t& fromActivityId, uint32_t& fromPointId, const uint32_t& numberOfOutputs) const;
		/*!
		 * \param fromActivityId, fromPointId It is a reference to an initial activity/location identification respectively.
		 * \param numberOfSeq Number of input or output parts.
		 * \param op It determines the type of a block, i.e. "assembly" or "disassembly".
		 * \return A complex block which type is specified by op parameter.
		 * \brief It is an auxiliary method wrapping complex calls to createBlockOfOperations method.
		 * \note It greatly simplifies the implementation of createAssemblyBlock and createDisassemblyBlock methods.
		 * \see mCompositeBlock, createAssemblyBlock, createDisassemblyBlock, createBlockOfOperations
		 */
		mCompositeBlock createComplexBlockHelper(uint32_t& fromActivityId, uint32_t& fromPointId, const uint32_t& numberOfSeq, const std::string& op) const;

		/*!
		 * \param fromActivityId, fromPointId It is a reference to an initial activity/location identification respectively.
		 * \param t1 The type of sub-block input activities, not necessarily IN.
		 * \param t2 The type of sub-block output activities, not necessarily OUT.
		 * \param numSeq Interval of the number of independent fixed sequences of some work, e.g. welding, cutting, etc.
		 * \param seqLength Interval of possible lengths of fixed sequences, i.e. e.g. atomic welding/cutting sub-tasks.
		 * \return A sub-block with alternatives, i.e. order of fixed sequences can be arbitrarily selected, used in complex blocks.
		 * \brief It is a core method for the decomposition of the robot work to well-defined blocks.
		 * \warning It updates fromActivityId and fromPointId values.
		 * \see mCompositeBlock, ActivityType
		 */
		mCompositeBlock createBlockOfOperations(uint32_t& fromActivityId, uint32_t& fromPointId, const ActivityType& t1,
				const ActivityType& t2, const Interval<uint32_t>& numSeq, const Interval<uint32_t>& seqLength) const;
		/*!
		 * \param fromActivityId An initial unique identification of new activities that will be updated.
		 * \param b A block to which interconnecting dynamic activities ('from -> to') are added.
		 * \param from, to Static activities between which the interconnections will be established.
		 * \param fullyConnected It influences the number of interconnections (true ~ all-to-all, false ~ any-to-any).
		 * \brief It interconnects two sub-blocks and adds the generated dynamic activities to the block b.
		 */
		void connect(uint32_t& fromActivityId, mCompositeBlock& b, const std::vector<StaticActivity*>& from,
				const std::vector<StaticActivity*>& to, bool fullyConnected = true) const;

		/*!
		 * \param blocks The blocks to be merged to one.
		 * \return The composite block created from blocks.
		 * \brief It copies the activities (their pointers) in blocks to the one composite block.
		 * \see mCompositeBlock
		 */
		mCompositeBlock mergeBlocks(const std::vector<mCompositeBlock>& blocks) const;


		//! Desired properties of generated instances defined in the input configuration file.
		ProjectParameters mParameters;
		//! A collection of activities belonging to the robot.
		std::vector<Activity*> mActivities;
		//! Available power saving modes for this robot.
		std::vector<RobotPowerMode*> mRobotModes;
};

/*!
 * The synchronization of robots is accomplished by using time lags.
 * Multiple instances of this class can model e.g. a workpiece/weldment
 * passing, which can be carried out either directly, i.e. gripper-to-gripper, or by using the bench.
 * \brief Instance of TimeLag class defines a time relation between two different robots.
 */
class TimeLag {
	public:
		/*!
		 * \param from, to A directed arc 'from -> to'.
		 * \param length The time offset on the arc which is shorter than cycle time.
		 * \param height It determines the offset in the number of cycles.
		 * \brief It initializes the time lag between two activities.
		 */
		TimeLag(Activity* from, Activity* to, const double& length, const int32_t& height)
			: mFrom(from), mTo(to), mLength(length), mHeight(height) { }

		Activity* from() const { return mFrom; }
		Activity* to() const { return mTo; }
		double length() const { return mLength; }
		int32_t height() const { return mHeight; }
	private:
		//! The activity from which the arc is leaving.
		Activity *mFrom;
		//! The activity to which the arc is entering.
		Activity *mTo;
		//! The time offset of the inter-robot arc.
		double mLength;
		//! The time offset in the number of cycles.
		int32_t mHeight;
};

/*!
 * \brief Auxiliary class encapsulating two activities to form collision or spatial compatibility pair.
 * \see InterRobotOperation::mSpatialCompatibility, RoboticLine::mCollisions, std::hash<Pair>
 */
class Pair {
	public:
		/*!
		 * \param activity1, mode1 First activity with its mode, i.e. a movement or location identification.
		 * \param activity2, mode2 Second activity with its mode, i.e. a movement or location identification.
		 * \brief It initializes the structure representing the pair of activities.
		 */
		Pair(Activity* activity1, const uint32_t& mode1, Activity* activity2, const uint32_t& mode2) :
			mActivity1(activity1), mActivity2(activity2), mMode1(mode1), mMode2(mode2) { }

		Activity* activity1() const { return mActivity1; }
		uint32_t mode1() const { return mMode1; }
		Activity* activity2() const { return mActivity2; }
		uint32_t mode2() const { return mMode2; }

		/*!
		 * \param p Pair p with which the current Pair, i.e. *this, is compared to.
		 * \return True if the pairs are equal otherwise false.
		 * \brief It compares two Pair data-structures and returns the result of comparison.
		 */
		bool operator==(const Pair& p) const;

	private:

		//! First activity.
		Activity *mActivity1;
		//! Second activity.
		Activity *mActivity2;
		//! The mode of the first activity, i.e. a movement or location identification.
		uint32_t mMode1;
		//! The mode of the second activity, i.e. a movement or location identification.
		uint32_t mMode2;
};

namespace std {
	/*!
	 * \brief A specialisation of std::hash template class for Pair data-structure.
	 */
	template <>
	struct hash<Pair>	{
		/*!
		 * \param p Pair data-structure from which the hash is computed.
		 * \return The hash value of p.
		 * \brief It calculates the hash value for the given argument.
		 */
		size_t operator() (const Pair& p) const {
			hash<Activity*> ptrHash;
			hash<uint32_t> modeHash;
			return ptrHash(p.activity1())^ptrHash(p.activity2())^modeHash(p.mode1())^modeHash(p.mode2());
		}
	};
}

/*!
 * The class corresponds to the inter-robot operation, e.g. a workpiece/weldment passing.
 * It incorporates time lags and spatial compatibility pairs that are necessary for the correct synchronisation between robots.
 * \brief The inter-robot operation corresponding to the workpiece/weldment handling.
 * \note Thanks to the time lags and compatibility pairs it is possible to model even more complex operations than there are currently modelled.
 */
class InterRobotOperation {
	public:
		/*!
		 * \param oid Integer identification of the operation.
		 * \param out The activity corresponds to the putting a weldment/workpiece on the bench or handing it over to another robot.
		 * \param in The activity corresponds to the getting a workpiece/weldment from the bench or another robot.
		 * \param par Parameters used for the generation of the inter-robot operation.
		 * \brief It creates the inter-robot operation according the desired parameters stated in ProjectParameters data-structure.
		 */
		InterRobotOperation(uint32_t oid, Activity* out, Activity* in, const ProjectParameters& par);

		uint32_t oid() const { return mOid; }
		std::string name() const { return mName; }
		void name(const std::string& name) { mName = name; }
		std::vector<TimeLag> timeLags() const { return mTimeLags; }
		void timeLags(const std::vector<TimeLag>& timeLags) { mTimeLags = timeLags; }
		std::vector<Pair> spatialCompatibility() const { return mSpatialCompatibility; }

	private:

		/*!
		 * \param a1, a2 Two activities between which the spatial compatibility must be met.
		 * \brief It is an auxiliary method for the constructor that helps to generate the compatibility pairs.
		 */
		void generateSpatialCompatibilityPairs(Activity* a1, Activity* a2);

		//! Integer identification of the inter-robot operation.
		uint32_t mOid;
		//! The name of the inter-robot operation.
		std::string mName;
		//! Time lags ensuring the correct time synchronization between robots.
		std::vector<TimeLag> mTimeLags;
		//! Spatial compatibility pairs, i.e. a handover takes place at the right location for both involved robots.
		std::vector<Pair> mSpatialCompatibility;
};

/*!
 * The instance of this class corresponds to the whole robotic cell including its robots, operations, and synchronisations.
 * Moreover, it holds the calculated production cycle time with respect to the generated data.
 * \brief The robotic cell corresponds to an instance of this class.
 */
class RoboticLine {
	public:
		/*!
		 * \param par Desired parameters of the robotic cell to be generated.
		 * \brief It generates all the required data-structures for the robotic cell, i.e. an instance of a robotic cell.
		 */
		RoboticLine(const ProjectParameters& par);

		std::vector<Robot*> robots() const { return mRobots; }
		std::vector<InterRobotOperation> interRobotOperations() const { return mInterRobotOperations; }
		std::vector<Pair> collisions() const { return mCollisions; }
		double productionCycleTime() const { return mProductionCycleTime; }
		std::vector<std::string> warnings() const { return mWarnings; }

		//! It frees all the memory occupied by robots.
		~RoboticLine();
	private:
		RoboticLine(const RoboticLine&) = delete;
		RoboticLine& operator=(const RoboticLine&) = delete;

		/*!
		 * \param par Desired properties of the generated collisions.
		 * \brief It generates collisions between robots.
		 */
		void generateCollisions(const ProjectParameters& par);
		/*!
		 * \param par Used to get a factor by which the lower estimation of the production cycle time is multiplied.
		 * \brief It calculates production cycle time with respect to the desired dilatation factor.
		 */
		void calculateProductionCycleTime(const ProjectParameters& par);

		//! Robots incorporated in the robotic cell.
		std::vector<Robot*> mRobots;
		//! Inter-robot operations for a weldment/workpiece passing.
		std::vector<InterRobotOperation> mInterRobotOperations;
		//! Collisions between robots.
		std::vector<Pair> mCollisions;
		//! The calculated production cycle time for *this robotic cell.
		double mProductionCycleTime;

		//! Warnings produced during the generation of this robotic cell. Written to the console later to not mix with the progress bar output.
		std::vector<std::string> mWarnings;
};

#endif
