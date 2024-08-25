#include "ArmRobotKinematics.hpp"
#include "Frame.hpp"

Frame* ArmRobotKinematics::addFrame(
        JointType jointType,
        double thetaFix,  
        double d,         
        double a,         
        double alphaFix   
    ) {
    // Use emplace_back to construct Frame in place
    m_frames.emplace_back(jointType, thetaFix, d, a, alphaFix);
    m_numFrames++;
    // Return a pointer to the newly added Frame
    // Note: No direct way to get the pointer, so we use reference to the last added frame
    return &m_frames.back();
}

void ArmRobotKinematics::forwardKinematics(){
    m_T = Eigen::Matrix4d::Identity();  // Initialize transformation matrix as identity

    for (auto& frame : m_frames) {
        m_T = m_T * frame.createTransformationMatrix();
    }

    m_pose.x = m_T(0, 3);
    m_pose.y = m_T(1, 3);
    m_pose.z = m_T(2, 3);

    m_pose.roll = std::atan2(m_T(2, 1), m_T(2, 2));
    m_pose.pitch = std::atan2(-m_T(2, 0), std::sqrt(m_T(2, 1) * m_T(2, 1) + m_T(2, 2) * m_T(2, 2)));
    m_pose.yaw = std::atan2(m_T(1, 0), m_T(0, 0));
}




vector<double> ArmRobotKinematics::inverseKinematics(Pose target_pose){
    vector<double> jointValues;

    // Target position and orientation
    Eigen::Vector3d target_position(target_pose.x, target_pose.y, target_pose.z);
    Eigen::Vector3d target_orientation(target_pose.roll, target_pose.pitch, target_pose.yaw);

    // Initialize variables
    Eigen::Vector3d position_error = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d orientation_error = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(m_numFrames);
    size_t iterations = 0;
    Eigen::VectorXd prev_dq = Eigen::VectorXd::Zero(m_numFrames);

    // While error is greater than tolerance, iterate
    while ((position_error.norm() > m_params.posTolerance || orientation_error.norm() > m_params.orTolerance) 
            && iterations < m_params.maxIterations){

        this->forwardKinematics();
        
        // Get the current position and orientation
        Eigen::Vector3d current_position(m_pose.x, m_pose.y, m_pose.z);
        Eigen::Vector3d current_orientation(m_pose.roll, m_pose.pitch, m_pose.yaw);

        // Calculate position error
        Eigen::Vector3d position_error = target_position - current_position;

        // Calculate orientation error
        Eigen::Vector3d orientation_error = target_orientation - current_orientation;

        this->jacobian();

        // Calculate condition number of the Jacobian matrix
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(m_jacobian);
        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

        double scaled_increment = m_params.increment / cond;

        // Compute the concatenated error vector and scale it
        Eigen::VectorXd concatenated_error(6);
        concatenated_error.head<3>() = position_error * scaled_increment;
        concatenated_error.tail<3>() = orientation_error * scaled_increment;

        // Solve for joint increments (dq)
        dq = m_jacobianPseudoInverse * concatenated_error + m_params.momentum * prev_dq;

        // Update previous dq
        prev_dq = dq;

        // Apply joint increments to update joint angles
        for(int i=0; i<m_numFrames; i++){
            m_frames[i].moveJoint(m_frames[i].getTheta() + dq(i));
        }
        
        iterations++;
        // Check if maximum iterations are reached and throw an error if so
        if (iterations >= m_params.maxIterations) {
            throw std::runtime_error("Inverse kinematics did not converge.");
        }
    }

    // Collect final joint values
    for(int i=0; i<m_numFrames; i++){
        Frame* p_frame = &m_frames[i];
        double theta;
        
        if(p_frame->getJointType() == REVOLUTE){
            theta = p_frame->getTheta();
            // Get the joint angle and ensure it is within [-PI, PI]
            if(theta > M_PI){
                theta -= 2*M_PI;
            }
            jointValues.push_back(theta);
        }
        else if(p_frame->getJointType() == PRISMATIC){
            // Get the prismatic joint displacement
            jointValues.push_back(p_frame->getD());
        }
    }

    return jointValues;
}



void ArmRobotKinematics::jacobian() {
    // Initialize Jacobian matrix with size 6 x number of frames
    Eigen::MatrixXd J(6, m_frames.size());
    J.setZero();  // Initialize to zero

    this->forwardKinematics();

    Eigen::Vector3d On(m_pose.x, m_pose.y, m_pose.z); // Position of the end-effector in the base frame coordinate system

    for(int i=0; i<m_numFrames; i++){
        Frame* p_frame = &m_frames[i];
        int jointType = p_frame->getJointType();

        if(jointType == FIXED_ROTATION || jointType == FIXED_TRANSLATION){
            continue;
        }

        Eigen::Matrix4d T = p_frame->getTransformMatrix();
        
        // Get the rotation axis of the i-th joint (third column)
        Eigen::Vector3d Zi = T.col(2).head<3>();
        
        // Get the origin of the i-th frame (fourth column)
        Eigen::Vector3d Oi = T.col(3).head<3>();

        if(jointType == REVOLUTE){
            // Linear velocity for revolute joint
            J.block<3, 1>(0, i) = Zi.cross(On - Oi);
            // Angular velocity for revolute joint
            J.block<3, 1>(3, i) = Zi;
        }
        else if(jointType == PRISMATIC){
            // Linear velocity for prismatic joint
            J.block<3, 1>(0, i) = Zi;
            // Angular velocity for prismatic joint is zero
            J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
        }
        else {
            throw std::invalid_argument("Unknown joint type");
        }

    }
    m_jacobian = J;
    // Compute the pseudo-inverse of J
    Eigen::MatrixXd m_jacobianPseudoInverse = J.completeOrthogonalDecomposition().pseudoInverse();
}
