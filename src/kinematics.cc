#include <hal.h>

class HomogeneousTransform 
{
    public:
    Eigen::Matrix4f H;

    HomogeneousTransform();
    Eigen::Matrix4f translate(const Eigen::Vector3f& V);
    Eigen::Matrix4f rotate(const char axis, float theta);

    private:
    void xRot(float theta);
    void yRot(float theta);
    void zRot(float theta);
};

class ForwardKinematics
{
    public:
    ForwardKinematics();

    void get_robot_pos_H(const std::vector<float>& rotation);
    void parallel_linkage_compensation_fw();

    private:
    std::vector<Eigen::Vector3f> joint_lengths;
};

HomogeneousTransform::HomogeneousTransform(){
    H = Eigen::Matrix4f::Identity();
};

Eigen::Matrix4f HomogeneousTransform::translate(const Eigen::Vector3f& V) {
    H.block<3, 1>(0, 3) = V;
    return H;
}

Eigen::Matrix4f HomogeneousTransform::rotate(const char axis, float theta){
    switch (axis) {
        case 'X':
            xRot(theta);
            break;
        case 'Y':
            yRot(theta);
            break;
        case 'Z':
            zRot(theta);
            break;
        default:
            break;
    }
    return H;
}

void HomogeneousTransform::xRot(float theta){
    Eigen::Matrix3f R;
    R << 1, 0, 0,
            0, std::cos(theta), -std::sin(theta),
            0, std::sin(theta), std::cos(theta);
    H.block<3, 3>(0, 0) = R;
}

void HomogeneousTransform::yRot(float theta){
    Eigen::Matrix3f R;
    R << std::cos(theta), 0, std::sin(theta),
            0, 1, 0,
            -std::sin(theta), 0, std::cos(theta);
    H.block<3, 3>(0, 0) = R;
}

void HomogeneousTransform::zRot(float theta){
    Eigen::Matrix3f R;
    R << std::cos(theta), -std::sin(theta), 0,
            std::sin(theta), std::cos(theta), 0,
            0, 0, 1;
    H.block<3, 3>(0, 0) = R;
}

ForwardKinematics::ForwardKinematics(){

}

void ForwardKinematics::get_robot_pos_H(const std::vector<float>& rotation){
        // Translation and rotation for each joint
        HomogeneousTransform R0, T0, R1, T1, R2, T2, R3, T3, R4, T4, R5, T5;
        R0.rotate('Z', rotation[0]);
        T0.translate(joint_lengths[0]);

        R1.rotate('Y', rotation[1]);
        T1.translate(joint_lengths[1]);

        R2.rotate('Y', rotation[2]);
        T2.translate(joint_lengths[2]);

        T3.translate(joint_lengths[3]);

        R4.rotate('Y', rotation[4]);
        T4.translate(joint_lengths[4]);
        
        R5.rotate('X', rotation[5]);
        T5.translate(joint_lengths[5]);

        // Compute the final transformation matrix
        Eigen::Matrix4f H = R0.H * T0.H * R1.H * T1.H * R2.H * T2.H * R3.H * T3.H * R4.H * T4.H * R5.H * T5.H;
}