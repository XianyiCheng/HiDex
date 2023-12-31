#include "optimizer.h"
#include "sdf.h"

const double lambdaDistance = 1.0;
const double lambdaPrior = 0;
const double lambdaNormal = 0.1;
const double lambdaRepell = 0.01;

using namespace Eigen;
using namespace std;

double signedDistance(SkeletonPtr manipulator, const std::string &part_name, const Vector3d &p1)
{
    BodyNodePtr mbn = manipulator->getBodyNode(part_name);
    Isometry3d mtf = mbn->getWorldTransform();
    ShapePtr ms = mbn->getShapeNodes().front()->getShape();
    MeshShape *meshShape =
        dynamic_cast<MeshShape *>(ms.get());

    // Find closest point to p on the mesh
    int closest_idx = 0;
    double closest_dist = 1000000;
    Vector3d p_closest;
    Vector3d n_closest;
    // for (int i = 0; i < meshShape->getMesh()->mMeshes[0]->mNumVertices; i++)
    for (int i = 0; i < 10; i++)
    {
        aiVector3D point = meshShape->getMesh()->mMeshes[0]->mVertices[i];
        Vector3d p;
        p << point[0], point[1], point[2];
        Vector3d p_w = mtf * p;

        double d = (p_w - p1).norm();
        if (d < closest_dist)
        {
            aiVector3D normal = meshShape->getMesh()->mMeshes[0]->mNormals[i];
            Vector3d n;
            n << normal[0], normal[1], normal[2];
            Vector3d n_w = mtf.rotation() * n;

            closest_dist = d;
            closest_idx = i;
            n_closest = n_w;
            p_closest = p_w;
        }
    }

    // Find the sign
    Vector3d v = p1 - p_closest;
    double sign = v.dot(n_closest);
    if (sign > 0)
    {
        return closest_dist;
    }
    else
    {
        return -closest_dist;
    }
}

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

        Vector3d p_w = mtf * p;
        Vector3d n_w = mtf.rotation() * n;

        mpts.push_back(ContactPoint(p_w, n_w));
    }

    return mpts;
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

        Vector3d handNormal = hand_contact_points[i].n;     // point outward
        Vector3d objectNormal = object_contact_points[i].n; // point inward

        Vector3d distanceVec = handPoint - objectPoint;

        // we want the normals to be as close to
        // perfectly aligned as possible
        double normalComp = 1 - handNormal.dot(objectNormal);

        distanceError += distanceVec.norm();
        normalError += normalComp * normalComp;
    }

    double wieghtedDistanceError = lambdaDistance * distanceError;
    double wieghtedNormalError = lambdaNormal * normalError;
    double wieghtedPriorError = lambdaPrior * priorError;

    double c = wieghtedDistanceError + wieghtedNormalError + wieghtedPriorError;

    return c;
}

double NumericalFunction::eval(const VectorXd &_x)
{
    return computeObjective(_x, true);
}

// Current method: numerical
void NumericalFunction::evalGradient(const VectorXd &_x, Map<VectorXd> _grad)
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

    mSolver->setNumMaxIterations(1000);

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

/////////////////////
double SDFOptFunction::computeObjective(const VectorXd &candidate,
                                        bool storeResults)
{

    // Update to potential solution
    mSetup->loadRobotConfig(candidate);

    // Compute attraction
    std::vector<ContactPoint> hand_contact_points = mSetup->getAttractManipulatorContactPointsWorld();
    std::vector<ContactPoint> object_contact_points = mSetup->getAttractContactPointsWorld();

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

        Vector3d handNormal = hand_contact_points[i].n;     // point outward
        Vector3d objectNormal = object_contact_points[i].n; // point inward

        Vector3d distanceVec = handPoint - objectPoint;

        // we want the normals to be as close to
        // perfectly aligned as possible
        double normalComp = 1 - handNormal.dot(objectNormal);

        distanceError += distanceVec.norm();
        normalError += normalComp * normalComp;
    }

    double wieghtedDistanceError = lambdaDistance * distanceError;
    double wieghtedNormalError = lambdaNormal * normalError;
    double wieghtedPriorError = lambdaPrior * priorError;

    double c_attract = wieghtedDistanceError + wieghtedNormalError + wieghtedPriorError;

    double d_repell = 0.0;
    
    // Compute repulsion: option 1, sdf for each repell finger part
    // {
    //     std::vector<ContactPoint> repell_object_contacts_world = mSetup->getRepellContactPointsWorld();
    //     std::vector<std::string> repell_hand_parts = mSetup->getRepellPartNames();

    //     SkeletonPtr manipulator = mSetup->getManipulator();

    //     for (auto repell_part : repell_hand_parts)
    //     {
    //         for (auto pt : repell_object_contacts_world)
    //         {
    //             double signed_d = signedDistance(manipulator, repell_part, pt.p);
    //             d_repell += signed_d;
    //         }
    //     }
    // }

    // Compute repulsion: option 2, sdf for the object
    // for each repell parts, have n_sample points and compute the signed distance to the object
    {
        std::vector<std::string> repell_hand_parts = mSetup->getRepellPartNames();
        std::vector<std::vector<int>> repell_sampled_idxes = mSetup->getRepellSampledIdxes();
        int n_rparts = repell_hand_parts.size();
        for (int i = 0; i < n_rparts; i++){
            std::string part_name = repell_hand_parts[i];
            std::vector<int> sampled_idxes = repell_sampled_idxes[i];

            std::vector<ContactPoint> repell_object_contacts_world = mSetup->getPointsWorld(part_name, sampled_idxes);
            for (auto pt : repell_object_contacts_world){
                double signed_d = BoxSDF(mSetup->getBoxShape(), mSetup->getObjectPose(), pt.p);
                d_repell += signed_d;
            }
        }
    }

    // maximize repulsion distance
    double c = c_attract - lambdaRepell * d_repell;

    return c;
}

////////////////////////////// SDF OPTIMIZER ///////////////////////////////////////

SDFOptimizer::SDFOptimizer(std::shared_ptr<SDFOptSetup> &setup)
{
    mSetup = setup;

    SkeletonPtr manipulator = setup->getManipulator();

    int dofs = manipulator->getNumDofs();

    mProblem = std::make_shared<Problem>(dofs);

    std::shared_ptr<SDFOptFunction> obj = std::make_shared<SDFOptFunction>(mSetup);

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
void SDFOptimizer::solve()
{
    mSolver = std::make_shared<NloptSolver>(mProblem, NloptSolver::LD_MMA);

    mSolver->setNumMaxIterations(1000);

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

pair<double, VectorXd> SDFOptimizer::getSolution()
{
    std::shared_ptr<Problem> problem = mSolver->getProblem();
    return make_pair(problem->getOptimumValue(), problem->getOptimalSolution());
}

std::vector<ContactPoint> BoxSurfaceOptSetup::getUpdatedObjectContactPointsWorld(const VectorXd &u){
    int n_contacts = getNumContactPoints();
    std::vector<ContactPoint> updatedContactPoints_local = mContactPoints_local;
    for (int i = 0; i < n_contacts; i++){
        for (int j = 0; j < 2; j++){
            int idx = pointUVDirections[i][j];
            updatedContactPoints_local[i].p[idx] = u[2*i + j];
        }
    }
    std::vector<ContactPoint> updatedContactPoints_world = transform_contact_points(updatedContactPoints_local, mObjectPose);
    return updatedContactPoints_world;
}

double BoxSurfaceOptFunction::computeObjective(const VectorXd &candidate,
                                     bool storeResults)
{

    // Update to potential solution
    // x: robot config, u1, v1, u2, v2, ... un, vn (for all object contact points)
    int n_robot_dofs = mSetup->getNumRobotDofs();
    int n_contacts = mSetup->getNumContactPoints();

    mSetup->loadRobotConfig(candidate.head(n_robot_dofs));

    std::vector<ContactPoint> hand_contact_points = mSetup->getManipulatorContactPointsWorld();
    std::vector<ContactPoint> object_contact_points = mSetup->getUpdatedObjectContactPointsWorld(candidate.tail(2*n_contacts));

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

        Vector3d handNormal = hand_contact_points[i].n;     // point outward
        Vector3d objectNormal = object_contact_points[i].n; // point inward

        Vector3d distanceVec = handPoint - objectPoint;

        // we want the normals to be as close to
        // perfectly aligned as possible
        double normalComp = 1 - handNormal.dot(objectNormal);

        distanceError += distanceVec.norm();
        normalError += normalComp * normalComp;
    }

    double wieghtedDistanceError = lambdaDistance * distanceError;
    double wieghtedNormalError = lambdaNormal * normalError;
    double wieghtedPriorError = lambdaPrior * priorError;

    double c = wieghtedDistanceError + wieghtedNormalError + wieghtedPriorError;

    return c;
}

BoxSurfaceOptimizer::BoxSurfaceOptimizer(std::shared_ptr<BoxSurfaceOptSetup> &setup)
{
    mSetup = setup;

    SkeletonPtr manipulator = setup->getManipulator();

    int dofs = manipulator->getNumDofs();
    int n_contacts = mSetup->getNumContactPoints();
    int n_var = dofs + 2*n_contacts;

    mProblem = std::make_shared<Problem>(n_var);

    std::shared_ptr<BoxSurfaceOptFunction> obj = std::make_shared<BoxSurfaceOptFunction>(mSetup);

    mProblem->setObjective(obj);

    VectorXd lowerLimits(n_var);
    VectorXd upperLimits(n_var);

    for (int i = 0; i < dofs; i++)
    {
        DegreeOfFreedom *dof = manipulator->getDof(i);
        lowerLimits[i] = dof->getPositionLowerLimit();
        upperLimits[i] = dof->getPositionUpperLimit();
    }
    for (int i = 0; i < n_contacts; i++ ){
        for (int k = 0; k < 2; k++){
            int idx = mSetup->pointUVDirections[i][k];
            lowerLimits[dofs + 2*i + k] = -mSetup->mBoxShape[idx]/2;
            upperLimits[dofs + 2*i + k] = mSetup->mBoxShape[idx]/2;
        }
    }

    mProblem->setLowerBounds(lowerLimits);
    mProblem->setUpperBounds(upperLimits);
}

// For gradient free, use LN_COBYLA. With gradient use LD_MMA
void BoxSurfaceOptimizer::solve()
{
    mSolver = std::make_shared<NloptSolver>(mProblem, NloptSolver::LD_MMA);
    // mSolver= std::make_shared<GradientDescentSolver>(mProblem);

    mSolver->setNumMaxIterations(10000);

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

pair<double, VectorXd> BoxSurfaceOptimizer::getSolution()
{
    std::shared_ptr<Problem> problem = mSolver->getProblem();
    return make_pair(problem->getOptimumValue(), problem->getOptimalSolution());
}


std::vector<ContactPoint> ObjectSurfaceOptSetup::getUpdatedObjectContactPointsLocal(const VectorXd &uv){
    int n_contacts = getNumContactPoints();
    std::vector<ContactPoint> updatedContactPoints_local;
    for (int i = 0; i < n_contacts; i++){
        // ContactPoint cp = this->expmap_mesh->exp_map(this->object_contact_idxes[i], uv[2*i], uv[2*i+1]);
        ContactPoint cp = this->expmap_mesh->exp_map_uv(this->object_contact_idxes[i], uv[2*i], uv[2*i+1]);
        updatedContactPoints_local.push_back(cp);
    }
    return updatedContactPoints_local;
}

std::vector<ContactPoint> ObjectSurfaceOptSetup::getUpdatedObjectContactPointsWorld(const VectorXd &uv){
    int n_contacts = getNumContactPoints();
    std::vector<ContactPoint> updatedContactPoints_local;
    for (int i = 0; i < n_contacts; i++){
        // ContactPoint cp = this->expmap_mesh->exp_map(this->object_contact_idxes[i], uv[2*i], uv[2*i+1]);
        ContactPoint cp = this->expmap_mesh->exp_map_uv(this->object_contact_idxes[i], uv[2*i], uv[2*i+1]);
        updatedContactPoints_local.push_back(cp);
    }
    std::vector<ContactPoint> updatedContactPoints_world = transform_contact_points(updatedContactPoints_local, mObjectPose);
    return updatedContactPoints_world;
}

double ObjectSurfaceOptFunction::computeObjective(const VectorXd &candidate,
                                     bool storeResults)
{

    // Update to potential solution
    // x: robot config, u1, v1, u2, v2, ... un, vn (for all object contact points)
    int n_robot_dofs = mSetup->getNumRobotDofs();
    int n_contacts = mSetup->getNumContactPoints();

    mSetup->loadRobotConfig(candidate.head(n_robot_dofs));

    std::vector<ContactPoint> hand_contact_points = mSetup->getManipulatorContactPointsWorld();
    std::vector<ContactPoint> object_contact_points = mSetup->getUpdatedObjectContactPointsWorld(candidate.tail(2*n_contacts));

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

        Vector3d handNormal = hand_contact_points[i].n;     // point outward
        Vector3d objectNormal = object_contact_points[i].n; // point inward

        Vector3d distanceVec = handPoint - objectPoint;

        // we want the normals to be as close to
        // perfectly aligned as possible
        double normalComp = 1 - handNormal.dot(objectNormal);

        distanceError += distanceVec.norm();
        normalError += normalComp * normalComp;
    }

    double wieghtedDistanceError = lambdaDistance * distanceError;
    double wieghtedNormalError = lambdaNormal * normalError;
    double wieghtedPriorError = lambdaPrior * priorError;

    double c = wieghtedDistanceError + wieghtedNormalError + wieghtedPriorError;

    return c;
}

ObjectSurfaceOptimizer::ObjectSurfaceOptimizer(std::shared_ptr<ObjectSurfaceOptSetup> &setup)
{
    mSetup = setup;

    SkeletonPtr manipulator = setup->getManipulator();

    int dofs = manipulator->getNumDofs();
    int n_contacts = mSetup->getNumContactPoints();
    int n_var = dofs + 2*n_contacts;

    mProblem = std::make_shared<Problem>(n_var);

    std::shared_ptr<ObjectSurfaceOptFunction> obj = std::make_shared<ObjectSurfaceOptFunction>(mSetup);

    mProblem->setObjective(obj);

    // set limits

    VectorXd lowerLimits(n_var);
    VectorXd upperLimits(n_var);

    for (int i = 0; i < dofs; i++)
    {
        DegreeOfFreedom *dof = manipulator->getDof(i);
        lowerLimits[i] = dof->getPositionLowerLimit();
        upperLimits[i] = dof->getPositionUpperLimit();
    }

    double max_geo_dist = 0.1;
    VectorXd initialGuess = mSetup->getInitialGuess();
    for (int i = 0; i < n_contacts; i++ ){
        
        // // dist limits
        // lowerLimits[dofs + 2*i] = 1e-15;
        // upperLimits[dofs + 2*i] = max_geo_dist;
        // initialGuess[dofs + 2*i] = lowerLimits[dofs + 2*i];

        // // angle
        // lowerLimits[dofs + 2*i + 1] = -3.15;
        // upperLimits[dofs + 2*i + 1] = 3.15;
        // initialGuess[dofs + 2*i + 1] = 0.0;
        
        // uv
        lowerLimits[dofs + 2*i] = -max_geo_dist;
        upperLimits[dofs + 2*i] = max_geo_dist;
        lowerLimits[dofs + 2*i + 1] = -max_geo_dist;
        upperLimits[dofs + 2*i + 1] = max_geo_dist;
        initialGuess[dofs + 2*i] = 0.0;
        initialGuess[dofs + 2*i + 1] = 0.0;
    }
    mSetup->updateInitialGuess(initialGuess);

    mProblem->setLowerBounds(lowerLimits);
    mProblem->setUpperBounds(upperLimits);

    double initial_obj = obj->computeObjective(initialGuess);
    std::cout << "Initial objective: " << initial_obj << std::endl;
    
}

// For gradient free, use LN_COBYLA. With gradient use LD_MMA
void ObjectSurfaceOptimizer::solve()
{
    mSolver = std::make_shared<NloptSolver>(mProblem, NloptSolver::LD_MMA);
    // mSolver= std::make_shared<GradientDescentSolver>(mProblem);

    mSolver->setNumMaxIterations(10000);

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

pair<double, VectorXd> ObjectSurfaceOptimizer::getSolution()
{
    std::shared_ptr<Problem> problem = mSolver->getProblem();
    return make_pair(problem->getOptimumValue(), problem->getOptimalSolution());
}