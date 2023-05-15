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

        // transform contact points to world frame
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

class SDFOptSetup
{
public:
    SDFOptSetup(SkeletonPtr manipulator, const std::vector<std::string> &AttractPartNames,
                const std::vector<int> &AttractPartPointIdxes,
                const std::vector<ContactPoint> &AttractContactPoints_local,
                const std::vector<std::string> &RepellPartNames,
                const std::vector<ContactPoint> &RepellContactPoints_local,
                const Vector7d &object_pose, const VectorXd &initial_guess, const Vector3d &box_shape, int n_sample = 20)
    {
        mManipulator = manipulator;
        mAttractPartNames = AttractPartNames;
        mAttractPartPointIdxes = AttractPartPointIdxes;
        mAttractContactPoints_local = AttractContactPoints_local;
        mRepellPartNames = RepellPartNames;
        mRepellContactPoints_local = RepellContactPoints_local; // repelling points on the object surface
        mObjectPose = object_pose;
        mInitialGuess = initial_guess;

        mBoxShape = box_shape;

        mAttractContactPoints_world = transform_contact_points(mAttractContactPoints_local, object_pose);
        mRepellContactPoints_world = transform_contact_points(mRepellContactPoints_local, object_pose);

        // n_sample: sampled points on each repelling parts for checking the object sdf

        for (auto rPart : RepellPartNames)
        {
            BodyNodePtr mbn = mManipulator->getBodyNode(rPart);
            ShapePtr ms = mbn->getShapeNodes().front()->getShape();
            MeshShape *meshShape =
                dynamic_cast<MeshShape *>(ms.get());
            int n_total = meshShape->getMesh()->mMeshes[0]->mNumVertices;
            std::vector<int> mIdxes;
            for (int k = 0; k < n_sample; k++)
            {
                mIdxes.push_back(randi(n_total));
            }
            mRepellSampledIdxes.push_back(mIdxes);
        }
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

    std::vector<ContactPoint> getPointsWorld(const std::string &part_name, std::vector<int> idxes)
    {
        std::vector<ContactPoint> pts;
        
        BodyNodePtr mbn = mManipulator->getBodyNode(part_name);
        Isometry3d mtf = mbn->getWorldTransform();
        ShapePtr ms = mbn->getShapeNodes().front()->getShape();
        MeshShape *meshShape =
            dynamic_cast<MeshShape *>(ms.get());

        for (int idx : idxes)
        {
            Vector3d p;
            Vector3d n;
            {
                aiVector3D point = meshShape->getMesh()->mMeshes[0]->mVertices[idx];
                aiVector3D normal = meshShape->getMesh()->mMeshes[0]->mNormals[idx];
                p << point[0], point[1], point[2];
                n << normal[0], normal[1], normal[2];
            }
            Vector3d p_w = mtf * p;
            Vector3d n_w = mtf.rotation() * n;

            pts.push_back(ContactPoint(p_w, n_w));
        }
        return pts;
    }

    std::vector<ContactPoint> getAttractManipulatorContactPointsWorld()
    {

        std::vector<ContactPoint> mpts;

        for (int i = 0; i < mAttractPartNames.size(); i++)
        {

            BodyNodePtr mbn = mManipulator->getBodyNode(mAttractPartNames[i]);
            Isometry3d mtf = mbn->getWorldTransform();
            ShapePtr ms = mbn->getShapeNodes().front()->getShape();
            MeshShape *meshShape =
                dynamic_cast<MeshShape *>(ms.get());

            Vector3d p;
            Vector3d n;

            {
                int index = mAttractPartPointIdxes[i];
                aiVector3D point = meshShape->getMesh()->mMeshes[0]->mVertices[index];
                aiVector3D normal = meshShape->getMesh()->mMeshes[0]->mNormals[index];
                p << point[0], point[1], point[2];
                n << normal[0], normal[1], normal[2];
            }

            Vector3d p_w = mtf * p;
            Vector3d n_w = mtf.rotation() * n;

            mpts.push_back(ContactPoint(p_w, n_w));
        }
        return mpts;
    }

    std::vector<ContactPoint> getAttractContactPointsWorld()
    {
        return mAttractContactPoints_world;
    }

    std::vector<ContactPoint> getRepellContactPointsWorld()
    {
        return mRepellContactPoints_world;
    }

    std::vector<std::string> getRepellPartNames()
    {
        return mRepellPartNames;
    }

    std::vector<std::vector<int>> getRepellSampledIdxes()
    {
        return mRepellSampledIdxes;
    }

    Vector3d getBoxShape()
    {
        return mBoxShape;
    }
    Vector7d getObjectPose()
    {
        return mObjectPose;
    }

protected:
    SkeletonPtr mManipulator;

    // Attraction points
    std::vector<std::string> mAttractPartNames;
    std::vector<int> mAttractPartPointIdxes;
    std::vector<ContactPoint> mAttractContactPoints_local;
    std::vector<ContactPoint> mAttractContactPoints_world;

    // Repelling Points, each repell part should repell all the repell contact points
    std::vector<std::string> mRepellPartNames;
    std::vector<ContactPoint> mRepellContactPoints_local;
    std::vector<ContactPoint> mRepellContactPoints_world;
    std::vector<std::vector<int>> mRepellSampledIdxes; // sampled points on each repelling parts for checking the object sdf
    Vector3d mBoxShape;

    Vector7d mObjectPose;
    VectorXd mInitialGuess;

    int n_sampled_points;
};

class SDFOptFunction : public Function
{
public:
    SDFOptFunction(std::shared_ptr<SDFOptSetup> &setup)
    {
        mSetup = setup;
    }

    double computeObjective(const VectorXd &candidate,
                            bool storeResults = false);

    /// _x is the DOF vector
    double eval(const VectorXd &_x) override
    {
        return computeObjective(_x, true);
    }

    /// _x is the DOF vector, _grad will store the gradient of the objective
    /// function after evaluation
    void evalGradient(const VectorXd &_x, Map<VectorXd> _grad) override
    {
        double currentObjectiveValue = computeObjective(_x);

        int dofs = _x.size();

        double step = 0.0001;

        // EXPENSIVE!! Currently uses forward differencing
        for (int i = 0; i < dofs; i++)
        {
            VectorXd forwardDiff = _x;

            forwardDiff[i] += step;

            double forwardObjective = computeObjective(forwardDiff);
            double JthetaDof = (forwardObjective - currentObjectiveValue) / step;

            _grad[i] = JthetaDof;
        }
    }

protected:
    /// Main Controller - needed to compute objective function in current
    /// iteration
    std::shared_ptr<SDFOptSetup> mSetup;
};

class SDFOptimizer
{
public:
    SDFOptimizer(std::shared_ptr<SDFOptSetup> &setup);

    pair<double, VectorXd> getSolution();

    void solve();

protected:
    /// The problem to solve
    std::shared_ptr<Problem> mProblem;

    /// The solver used to solve the problem
    std::shared_ptr<Solver> mSolver;

    /// Main Setup - pass to optimization function
    std::shared_ptr<SDFOptSetup> mSetup;
};