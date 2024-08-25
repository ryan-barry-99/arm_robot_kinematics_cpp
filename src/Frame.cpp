#include "Frame.hpp"

Frame::Frame(
    JointType jointType, 
    double thetaFix, 
    double d, 
    double a, 
    double alphaFix
)
    :   m_jointType(jointType),
        m_thetaFix(thetaFix),
        m_theta(thetaFix),
        m_d(d),
        m_alphaFix(alphaFix),
        m_alpha(alphaFix)
{}

void Frame::moveJoint(double jointValue){
    if (m_jointType == REVOLUTE) {
        m_theta = fmod(jointValue, 2 * M_PI);
    } 
    else if (m_jointType == PRISMATIC) {
        if (m_d == 0) {
            m_a += jointValue;
        } else if (m_a == 0) {
            m_d += jointValue;
        }
    }
}

Eigen::Matrix4d Frame::createTransformationMatrix() {
    double ct = cos(m_thetaFix + m_theta);
    double st = sin(m_thetaFix + m_theta);
    double ca = cos(m_alpha);
    double sa = sin(m_alpha);

    m_transformMatrix <<    ct, -st * ca, st * sa, m_a * ct,
                            st, ct * ca, -ct * sa, m_a * st,
                            0, sa, ca, m_d,
                            0, 0, 0, 1;

    return m_transformMatrix;
}