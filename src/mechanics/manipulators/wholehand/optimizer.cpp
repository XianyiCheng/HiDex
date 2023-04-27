#include "optimizer.h"

const double lambdaDistance = 1.0;
const double lambdaPrior = 0.01;
const double lambdaNormal = 0.01;

using namespace Eigen;
using namespace std;

std::vector<ContactPoint> OptSetup::getManipulatorContactPointsWorld()
{

    std::vector<ContactPoint> mpts;

    for (int i = 0; i < mPartNames.size(); i++)
    {

        BodyNodePtr mbn = mManipulator->getBodyNode(mPartNames[i]);
        Isometry3d mtf = mbn->getWorldTransform();
        ShapePtr ms = mbn->getShapeNodes().front()->getShape();
        MeshShape *meshShape =
            dynamic_cast<MeshShape *>(ms.get());
        
        Vector3d p;
        Vector3d n;

        {
            int index = mPartPointIdxes[i];
            aiVector3D point = meshShape->getMesh()->mMeshes[0]->mVertices[index];
            aiVector3D normal = meshShape->getMesh()->mMeshes[0]->mNormals[index];
            p << point[0], point[1], point[2];
            n << normal[0], normal[1], normal[2];
        }
        p << 0,0,0;

        Vector3d p_w = mtf * p;
        Vector3d n_w = mtf.rotation() * n;
        
        mpts.push_back(ContactPoint(p_w, n_w));
    }

    return mpts;
}
OptFunction::OptFunction(std::shared_ptr<OptSetup> &setup)
{
    mSetup = setup;
}

double OptFunction::computeObjective(const VectorXd &candidate,
                                     bool storeResults)
{

    // Update to potential solution
    mSetup->loadRobotConfig(candidate);

    std::vector<ContactPoint> hand_contact_points = mSetup->getManipulatorContactPointsWorld();
    std::vector<ContactPoint> object_contact_points = mSetup->getObjectContactPointsWorld();

    double distanceError = 0.0;
    double normalError = 0.0;
    double priorError = 0.0;

    int numMappings = hand_contact_points.size();

    VectorXd priorDiff = candidate - this->mSetup->getInitialGuess();
    priorError += priorDiff.norm();

    for (int i = 0; i < numMappings; i++)
    {
        Vector3d handPoint = hand_contact_points[i].p;
        Vector3d objectPoint = object_contact_points[i].p;

        Vector3d handNormal = hand_contact_points[i].n;
        Vector3d objectNormal = object_contact_points[i].n;

        Vector3d distanceVec = handPoint - objectPoint;

        // we want the normals to be as close to
        // perfectly inverted as possible
        double normalComp = 1 + handNormal.dot(objectNormal);

        distanceError += distanceVec.norm();
        normalError += normalComp * normalComp;
    }

    double wieghtedDistanceError = lambdaDistance * distanceError;
    double wieghtedNormalError = lambdaNormal * normalError;
    double wieghtedPriorError = lambdaPrior * priorError;

    double c = wieghtedDistanceError + wieghtedNormalError + wieghtedPriorError;

    return c;
}

double OptFunction::eval(const VectorXd &_x)
{
    return computeObjective(_x, true);
}

// Current method: numerical
void OptFunction::evalGradient(const VectorXd &_x, Map<VectorXd> _grad)
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

////////////////////////////////////////////////////////////////////////////////////////

Optimizer::Optimizer(std::shared_ptr<OptSetup> &setup)
{
    mSetup = setup;

    SkeletonPtr manipulator = setup->getManipulator();

    int dofs = manipulator->getNumDofs();

    mProblem = std::make_shared<Problem>(dofs);

    std::shared_ptr<OptFunction> obj = std::make_shared<OptFunction>(mSetup);

    mProblem->setObjective(obj);

    VectorXd lowerLimits;
    VectorXd upperLimits;

    lowerLimits.resize(dofs);
    upperLimits.resize(dofs);

    for (int i = 0; i < dofs; i++)
    {
        DegreeOfFreedom *dof = manipulator->getDof(i);
        lowerLimits[i] = dof->getPositionLowerLimit();
        upperLimits[i] = dof->getPositionUpperLimit();
    }

    mProblem->setLowerBounds(lowerLimits);
    mProblem->setUpperBounds(upperLimits);
}

// For gradient free, use LN_COBYLA. With gradient use LD_MMA
void Optimizer::solve()
{
    mSolver = std::make_shared<NloptSolver>(mProblem, NloptSolver::LD_MMA);

    mSolver->setNumMaxIterations(500);

    // Set initial guess
    VectorXd initialGuess = mSetup->getInitialGuess();

    mSolver->getProblem()->setInitialGuess(initialGuess);

    auto start = chrono::steady_clock::now();

    bool solved = mSolver->solve();

    auto end = chrono::steady_clock::now();

    int diff = chrono::duration_cast<chrono::milliseconds>(end - start).count();

    cout << "Optimization finished in: " << diff << " milliseconds" << endl;

    if (!solved)
    {
        cout << "Unable to find solution" << endl;
    }
}

pair<double, VectorXd> Optimizer::getSolution()
{
    std::shared_ptr<Problem> problem = mSolver->getProblem();
    return make_pair(problem->getOptimumValue(), problem->getOptimalSolution());
}
