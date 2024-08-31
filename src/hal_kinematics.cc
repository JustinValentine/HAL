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
    ~ForwardKinematics();

    void get_robot_pos();
    void parallel_linkage_compensation_fw();
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

ForwardKinematics::ForwardKinematics(){
    
}

void ForwardKinematics::get_robot_pos(){

}