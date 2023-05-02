#include <dart/dart.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>

#ifndef UTILS_H
#define UTILS_H
#include "../../utilities/utilities.h"
#endif

#ifndef CONTACTCONSTRAINTS_H
#define CONTACTCONSTRAINTS_H
#include "../../contacts/contact_constraints.h"
#endif

using namespace dart::dynamics;
using namespace dart::optimizer;
using namespace std;

class OptSetup
{
public:
    OptSetup(SkeletonPtr manipulator, const std::vector<std::string> &partNames, const std::vector<int> &partPointIdxes, const std::vector<ContactPoint> &contactPoints, const Vector7d &object_pose, const VectorXd &initial_guess)
    {
        mManipulator = manipulator;
        mPartNames = partNames;
        mPartPointIdxes = partPointIdxes;
        mContactPoints_local = contactPoints; // normals pointing inward to the center of the object
        mObjectPose = object_pose;
        mInitialGuess = initial_guess;

        // TODO: transform contact points to world frame
        mContactPoints_world = transform_contact_points(mContactPoints_local, object_pose);
    }

    void updateManipulator(SkeletonPtr manipulator)
    {
        mManipulator = manipulator;
    }
    void updateParts(std::vector<std::string> partNames)
    {
        mPartNames = partNames;
    }
    void updatePartPointIdxes(std::vector<int> partPointIdxes)
    {
        mPartPointIdxes = partPointIdxes;
    }
    void updateContactPoints_n_ObjectPose(const std::vector<ContactPoint> &contactPoints, const Vector7d &object_pose)
    {
        mContactPoints_local = contactPoints;
        mObjectPose = object_pose;
        mContactPoints_world = transform_contact_points(mContactPoints_local, object_pose);
    }

    void updateObjectPose(const Vector7d &object_pose)
    {
        mObjectPose = object_pose;
        mContactPoints_world = transform_contact_points(mContactPoints_local, object_pose);
    }

    void updateInitialGuess(const VectorXd &initial_guess)
    {
        mInitialGuess = initial_guess;
    }

    SkeletonPtr getManipulator()
    {
        return mManipulator;
    }

    VectorXd getInitialGuess()
    {
        return mInitialGuess;
    }

    void loadRobotConfig(const VectorXd &robot_config)
    {
        mManipulator->setPositions(robot_config);
    }

    std::vector<ContactPoint> getManipulatorContactPointsWorld();

    std::vector<ContactPoint> getObjectContactPointsWorld()
    {
        return mContactPoints_world;
    }

protected:
    SkeletonPtr mManipulator;
    std::vector<std::string> mPartNames;
    std::vector<int> mPartPointIdxes;
    std::vector<ContactPoint> mContactPoints_local;
    Vector7d mObjectPose;
    VectorXd mInitialGuess;

    std::vector<ContactPoint> mContactPoints_world;
};

class OptFunction : public Function
{
public:
    OptFunction(std::shared_ptr<OptSetup> &setup);

    double computeObjective(const VectorXd &candidate,
                            bool storeResults = false);

    /// _x is the DOF vector
    double eval(const VectorXd &_x) override;

    /// _x is the DOF vector, _grad will store the gradient of the objective
    /// function after evaluation
    void evalGradient(const VectorXd &_x, Map<VectorXd> _grad) override;

protected:
    /// Main Controller - needed to compute objective function in current
    /// iteration
    std::shared_ptr<OptSetup> mSetup;
};

class Optimizer
{
public:
    Optimizer(std::shared_ptr<OptSetup> &setup);

    pair<double, VectorXd> getSolution();

    void solve();

protected:
    /// The problem to solve
    std::shared_ptr<Problem> mProblem;

    /// The solver used to solve the problem
    std::shared_ptr<Solver> mSolver;

    /// Main Setup - pass to optimization function
    std::shared_ptr<OptSetup> mSetup;
};