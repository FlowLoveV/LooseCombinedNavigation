//
// Created by 0-0 mashuo on 2023/6/4.
//

#include "LooseCombination.h"
#include "Rotation.h"
#include "Earth.h"

ns_GINS::LooseCombination::LooseCombination(const YAML::Node & config) {
    // 读取配置文件
    options_ = GinsOptions(config);
    obs_model = config["obsmodel"].as<int>();
    // 为IMU数据及结果数组均预留3个大小的长度，避免反复开辟内存
    imuData_.reserve(3);
    insState_.reserve(3);

    // 初始化滤波器初始状态和过程噪声
    Matrix dx = zero(RANK,1);
    Matrix Q = zero(NOISERANK,NOISERANK);
    // 生成一个lambda方法,便于修改Q
    double p[3] = {0};
    auto buildQ = [&Q](int begin,double p[], double factor){
        for (int i = begin; i < begin+3; ++i) {
            Q.assign(i,i,pow(p[i-begin],2)*factor);
        }
    };
    buildQ(VRW_ID, options_.imuNoise.accVrw , 1.0);
    buildQ(ARW_ID,options_.imuNoise.gyrArw,1.0);
    buildQ(BGSTD_ID,options_.imuNoise.gBiasStd,2/options_.imuNoise.corrTime[0]);
    buildQ(BASTD_ID,options_.imuNoise.aBiasStd,2/options_.imuNoise.corrTime[1]);
    buildQ(SGSTD_ID,options_.imuNoise.gScaleStd,2/options_.imuNoise.corrTime[2]);
    buildQ(SASTD_ID,options_.imuNoise.aScaleStd,2/options_.imuNoise.corrTime[3]);
    // 加入滤波器状态初值和imu过程噪声阵
    filter_.setMState0(dx);
    q_ = Q;

    // 初始化导航状态(位置、速度、姿态和IMU误差等信息)及其方差
    initialize();
}

void ns_GINS::LooseCombination::initialize() {
    // 初始化IMU位置状态
    INSRes_SingleEpoch res1;
    memcpy(res1.m_pPos,options_.initState.pos,3*sizeof(double));
    memcpy(res1.m_pSpeed,options_.initState.vel,3*sizeof(double));
    memcpy(res1.m_pEuler,options_.initState.euler,3*sizeof(double));
    Rotation::Euler2Quaternion(res1.m_pEuler,res1.m_pQuaternion);
    Rotation::Euler2EMatrix(res1.m_pEuler,res1.m_pEMatrix);
    res1.m_error = options_.initState.imuError;
    imuError_ = options_.initState.imuError;
    // 设置初始状态
    insState_.assign(3,res1);


    // 初始化状态方差阵
    // 创建一个lambda表达式
    auto buildP0 = [](Matrix &m,double *p){
        double t[3];
        for (int i = 0; i < 3; ++i) {
            t[i] = pow(p[i],2);
        }
        m = diag(t,3);
    };
    Matrix m[7];
    buildP0(m[0],options_.initStateStd.pos);
    buildP0(m[1],options_.initStateStd.vel);
    buildP0(m[2],options_.initStateStd.euler);
    buildP0(m[3],options_.initStateStd.imuError.gBias);
    buildP0(m[4],options_.initStateStd.imuError.aBias);
    buildP0(m[5],options_.initStateStd.imuError.gScale);
    buildP0(m[6],options_.initStateStd.imuError.aScale);
    // 将初始方差阵赋给滤波器
    filter_.setMStateVariance0(diag(m,7));
}

void ns_GINS::LooseCombination::addImuData(const IMUData_SingleEpoch &imudata, bool ifconpensate) {
    // 清除第k-2历元数据,增加新数据
    if(imuData_.size() < 3) imuData_.push_back(imudata);
    else imuData_[2] = imudata;
    /*imuData_.push_back(imudata);
    if(imuData_.size() > 3 ){
        imuData_.erase(imuData_.begin());
    }*/
    // imu误差补偿
    if(ifconpensate){
        double dt = imuData_[2].t - imuData_[1].t;
        imuData_[2].compensate(imuError_,dt);
    }
}

void ns_GINS::LooseCombination::addGnssResData(const GnssRes &gnssres) {
    gnssRes_ = gnssres;
    gnssRes_.m_isvalid = true;
}

void ns_GINS::LooseCombination::newProcess() {
    GPST updateTime = gnssRes_.m_isvalid ? gnssRes_.m_gpst : GPST();
    int flag = ifUpdate(updateTime);
    double dt;
    IMUData_SingleEpoch midImuData;
    INSRes_SingleEpoch midInsRes;
    switch (flag) {
        // 在上一历元更新
        case -1:
            // 先更新上一历元的IMU状态，反馈误差后再机械编排算法更新本历元IMU状态
            dt = imuData_[1].t - imuData_[0].t;
            // 为滤波构建矩阵
            preForPredict(insState_[0],imuData_[1],dt);
            preForUpdate(insState_[1],imuData_[1],dt);

            // 误差反馈给上一历元IMU
            insState_[1].stateFeedback(filter_.getMStateK());
            // 反馈误差后，机械编排算法更新IMU状态
            mech();
            // 滤波误差置零
            filter_.setMState0(zero(RANK,1));
            break;

        // 更新本历元
        case 0:
            dt = imuData_[2].t - imuData_[1].t;
            // 首先更新本历元IMU状态
            mech();
            preForPredict(insState_[1],imuData_[2],dt);
            preForUpdate(insState_[2],imuData_[2],dt);

            // 误差反馈给本历元IMU状态
            insState_[2].stateFeedback(filter_.getMStateK());
            // 滤波误差置零
            filter_.setMState0(zero(RANK,1));
            break;

        // 插值更新
        case 1:
            // 此时待更新历元在两个历元间,首先根据更新时间插值得到对应历元IMU数据
            imuData_[2].interpolationImuData(imuData_[1],midImuData,updateTime);
            // 这里不能使用mesh()函数
            // 首先补偿插值IMU数据
            dt = updateTime - imuData_[1].t;
            midImuData.compensate(imuError_,dt);
            // 机械编排算法更新IMU状态,并传递imu误差
            PureIns::updateSinEpoch(imuData_[0],imuData_[1],midImuData,
                                    insState_[0],insState_[1],midInsRes);
            midInsRes.m_error = imuError_;
            // 滤波更新状态
            preForPredict(insState_[1],midImuData,dt);
            preForUpdate(midInsRes,midImuData,dt);

            // 误差反馈
            midInsRes.stateFeedback(filter_.getMStateK());
            imuError_ = midInsRes.m_error;
            // 滤波误差置零
            filter_.setMState0(zero(RANK,1));
            // 接下来需要机械编排算法更新本历元IMU状态
            dt = imuData_[2].t - midImuData.t;
            imuData_[2].compensate(imuError_,dt);
            PureIns::updateSinEpoch(imuData_[1],midImuData,imuData_[2],
                                    insState_[1],midInsRes,insState_[2]);
            // 传递imu误差
            insState_[2].m_error = imuError_;
            break;
        // 机械编排推导
        case 2:
            dt = imuData_[2].t - imuData_[1].t;
            preForPredict(insState_[1],imuData_[2],dt);
            mech();
            break;
        default:
            std::exit(-1);
            break;
    }
    // 数据前推
    insState_[0] = insState_[1];
    insState_[1] = insState_[2];
    imuData_[0] = imuData_[1];
    imuData_[1] = imuData_[2];

    // test
}

NavState ns_GINS::LooseCombination::getNavState() {
    NavState state;
    memcpy(state.pos,insState_[2].m_pPos,3*sizeof(double));
    memcpy(state.vel,insState_[2].m_pSpeed,3*sizeof(double));
    memcpy(state.euler,insState_[2].m_pEuler,3*sizeof(double));
    state.imuError = insState_[2].m_error;
    return state;
}

void ns_GINS::LooseCombination::preForPredict(const INSRes_SingleEpoch & res,const IMUData_SingleEpoch & obs,const double &dt) {
    // 构建状态转移矩阵和系统噪声阵，进行一步预测

    // 姿态矩阵
    Matrix Cnb(3,3,res.m_pEMatrix);
    // G矩阵构建
    Matrix G_up = zero(RANK - NOISERANK,NOISERANK);
    Matrix G_down[6];
    G_down[0] = G_down[1] = Cnb;
    G_down[2] = G_down[3] = G_down[4] = G_down[5] = eye(3);
    Matrix G = vertical_stack(G_up, diag(G_down,6));


    // F矩阵构建
    // 关于Rm,Rn的变量
    double Rm=0,Rn=0,RmH=0,RnH=0,RmH2,RnH2,RmnH;
    // wie_n,wen_n
    double wie_n[3],wen_n[3];
    Earth::calculateRotationSpeed(res.m_pPos,res.m_pSpeed,wie_n,wen_n,Rm,Rn);
    RmH = Rm + res.m_pPos[2];   RmH2 = RmH * RmH;
    RnH = Rn + res.m_pPos[2];   RnH2 = RnH * RnH;
    RmnH = RmH * RnH;
    // n系相对于i系旋转角速度在n系下的投影
    Matrix win_n = Matrix(3,1,wie_n) + Matrix(3,1,wen_n);
    // 关于速度的变量
    double ve = res.m_pSpeed[0], vn = res.m_pSpeed[1], vd = res.m_pSpeed[2];
    // 关于纬度三角函数的变量
    double sinLat = sin(res.m_pPos[0]), cosLat = cos(res.m_pPos[0]);
    double tanLat = tan(res.m_pPos[0]), secLat = 1 / cosLat;
    // 比力，旋转角速度向量
    Matrix vector_fb = Matrix(3,1,obs.m_pAcc) / dt;
    Matrix vector_wib_b = Matrix(3,1,obs.m_pGyr) / dt;
    // 正常重力
    double gp = Earth::calculate_g(res.m_pPos[0],res.m_pPos[2]);

    // 3*3矩阵变量
    Matrix temp = zero(3,3);
    // lambda函数用于改变temp的值
    auto change_temp = [&temp](int row,int col,double value){
        temp.assign(row,col,value);
    };

    // F矩阵
    Matrix F = zero(RANK,RANK);
    Matrix I3 = eye(3);
    // lambda函数改变F矩阵
    // row,col代表子矩阵索引，m代表待赋值的矩阵
    auto change_F = [&F](int && row,int && col,const Matrix & m){
        int r = (row - 1) * 3 + 1;
        int c = (col - 1) * 3 + 1;
        for (int i = r; i < r+3; ++i) {
            for (int j = c; j < c+3; ++j) {
                F.assign(i,j,m(i-r+1,j-c+1));
            }
        }
    };

    // 位置误差转移
    // Frr 矩阵
    change_temp(1,1,-vd/RmH);
    change_temp(1,3,vn/RmH);
    change_temp(2,1,ve*tanLat/RnH);
    change_temp(2,2,-(vd+vn*tanLat)/RnH);
    change_temp(2,3,ve/RnH);
    change_F(1,1,temp);
    change_F(1,2,I3);

    // 速度误差转移
    temp.allSet(0);
    // Fvr矩阵
    change_temp(1,1,-2*ve*WGS84_WIE*cosLat/RmH - ve*ve*secLat*secLat/RmnH);
    change_temp(1,3,vn*vd/RmH2 - ve*ve*tanLat/RnH2);
    change_temp(2,1,2*WGS84_WIE*(vn*cosLat-vd*sinLat)/RmH + vn*ve*secLat*secLat/RmnH);
    change_temp(2,3,(ve*vd+vn*ve*tanLat)/RnH2);
    change_temp(3,1,2*WGS84_WIE*ve*sinLat/RmH);
    change_temp(3,3,-ve*ve/RnH2-vn*vn/RmH2+2*gp/(sqrt(Rm*Rn)+res.m_pPos[2]));
    change_F(2,1,temp);
    // Fvv矩阵
    temp.allSet(0);
    change_temp(1,1,vd/RmH);
    change_temp(1,2,-2*(WGS84_WIE*sinLat + ve*tanLat/RnH));
    change_temp(1,3,vn/RmH);
    change_temp(2,1,2*WGS84_WIE*sinLat+ve*tanLat/RnH);
    change_temp(2,2,(vd+vn*tanLat)/RnH);
    change_temp(2,3,2*WGS84_WIE*cosLat+ve/RnH);
    change_temp(3,1,-2*vn/RmH);
    change_temp(3,2,-2*(WGS84_WIE*cosLat+ve/RnH));
    change_F(2,2,temp);
    change_F(2,3, antiVector(Cnb * vector_fb));
    change_F(2,5,Cnb);
    change_F(2,7,Cnb * diag(vector_fb.p,3));


    // 姿态误差转移
    // Fphir
    temp.allSet(0);
    change_temp(1,1,-WGS84_WIE*sinLat/RmH);
    change_temp(1,3,ve/RnH2);
    change_temp(2,3,-vn/RmH2);
    change_temp(3,1,-WGS84_WIE*cosLat/RmH-ve*secLat*secLat/RmnH);
    change_temp(3,3,-ve*tanLat/RnH2);
    change_F(3,1,temp);
    // Fphiv
    temp.allSet(0);
    change_temp(1,2,1/RnH);
    change_temp(2,1,-1/RmH);
    change_temp(3,2,-tanLat/RnH);
    change_F(3,2,temp);
    change_F(3,3,-antiVector(win_n));
    change_F(3,4,-Cnb);
    change_F(3,6,-Cnb * diag(vector_wib_b.p,3));

    // IMU误差状态转移
    change_F(4,4, -1/options_.imuNoise.corrTime[0] * I3);
    change_F(5,5,- 1/options_.imuNoise.corrTime[1] * I3);
    change_F(6,6,- 1/options_.imuNoise.corrTime[2] * I3);
    change_F(7,7,- 1/options_.imuNoise.corrTime[3] * I3);


    // 状态转移矩阵
    Matrix phi =  F * dt + eye(RANK) ;
    // 系统噪声矩阵
    Matrix q = G * q_ * G.T();
    Matrix Q = 0.5 * dt * (phi * q * phi.T() + q);

    /*filter_.setMStateTrans(phi);
    filter_.setMSystemNoise(Q);*/
    filter_.Predict(phi,Q);
    filter_.m_StateVariance_k_1 = filter_.m_StateVariance_estimated;
}

void ns_GINS::LooseCombination::preForUpdate(const INSRes_SingleEpoch & res,const IMUData_SingleEpoch & obs,const double &dt) {
    // lambda函数，构建矩阵
    // lambda函数--用于改变分块矩阵的值
    auto changeBlock = [](Matrix & F,int && row,int && col,const Matrix & m){
        int r = (row - 1) * 3 + 1;
        int c = (col - 1) * 3 + 1;
        for (int i = r; i < r+3; ++i) {
            for (int j = c; j < c+3; ++j) {
                F.assign(i,j,m(i-r+1,j-c+1));
            }
        }
    };
    // lambda函数--用于生成三维向量的平方diag阵
    auto buildR = [](Matrix & R,double * p){
        for (int i = 0; i < 3; ++i) {
            R.assign(i+1,i+1,pow(p[i],2));
        }
    };

    // 生成观测值新息、量测矩阵和观测值噪声阵
    Matrix dz[obs_model],H[obs_model],R[obs_model];
    // 预备矩阵
    Matrix I3 = eye(3);
    Matrix Cnb(3,3,res.m_pEMatrix);
    Matrix lb(3,1,options_.AntennaLeverArm);
    Matrix Cnb_multiply_lb = Cnb * lb;
    // 使用本历元IMU状态

    // 构建GNSS位置观测方程
    // 将IMU位置转到GNSS天线相位中心位置
    Matrix DR = Earth::toDR(res.m_pPos[0],res.m_pPos[2]);
    Matrix DR_inv = Earth::toDRi(res.m_pPos[0],res.m_pPos[2]);
    Matrix antennaPos = Matrix(3,1,res.m_pPos) + DR_inv * Cnb_multiply_lb;
    // 位置观测值新息
    Matrix pos_dz = DR * (antennaPos - Matrix(3,1,gnssRes_.m_pBLH));
    // 位置观测矩阵
    Matrix pos_H = zero(3,RANK);
    changeBlock(pos_H,1,1,I3);
    changeBlock(pos_H,1,3,antiVector(Cnb_multiply_lb));
    // 位置方差阵
    Matrix pos_R = zero(3,3);
    buildR(pos_R,gnssRes_.m_pBLHStd);
    // 将位置观测相关矩阵放入矩阵数组
    dz[POS-1] = pos_dz;     H[POS-1] = pos_H;       R[POS-1] = pos_R;
    // 生成速度观测值的新息、矩阵和方差阵
    if(obs_model > POS){
        // n系相对于i系旋转角速度在n系下投影
        double p_win_n[3];
        Earth::calculateOmega_in_n(res.m_pPos,res.m_pSpeed,p_win_n);
        Matrix vector_win_n(3,1,p_win_n);
        Matrix anti_win_n = antiVector(vector_win_n);
        // imu三轴旋转角速度
        Matrix omega_ib_b = Matrix(3,1,obs.m_pGyr) / dt;
        // lb的反对称矩阵
        Matrix anti_lb = antiVector(lb);

        // 计算速度观测
        Matrix vel_dz = Matrix(3,1,res.m_pSpeed) - anti_win_n * Cnb_multiply_lb - Cnb * cross(lb,omega_ib_b) - Matrix(3,1,gnssRes_.m_pVned);
        // 计算速度观测矩阵
        Matrix vel_H = zero(3,RANK);
        Matrix Hv3 = - anti_win_n * antiVector(Cnb * lb) - antiVector(Cnb * cross(lb,omega_ib_b));
        Matrix Hv6 = - Cnb * anti_lb * diag(omega_ib_b.p,3);
        changeBlock(vel_H,1,2,I3);
        changeBlock(vel_H,1,3,Hv3);
        changeBlock(vel_H,1,4,-antiVector(Cnb * lb));
        changeBlock(vel_H,1,6,Hv6);
        // 计算速度方差阵
        Matrix vel_R = zero(3,3);
        buildR(vel_R,gnssRes_.m_pVnedStd);
        // 赋值
        dz[POS_VEL-1] = vel_dz;       H[POS_VEL-1] = vel_H;        R[POS_VEL-1] = vel_R;
    }


    // 生成里程计观测值的新息、矩阵和方差阵
    if(obs_model > POS_VEL){
        // 在这里添加代码
    }
    // 生成观测矩阵
    Matrix z = vertical_stack_array(dz,obs_model);
    Matrix h = vertical_stack_array(H,obs_model);
    Matrix r = diag(R,obs_model);
    /*filter_.setMObservation(vertical_stack_array(dz,obs_model));
    filter_.setMMeasurement(vertical_stack_array(H,obs_model));
    filter_.setMMeasureNoise(diag(R,obs_model));*/
    filter_.Update(z,h,r);

}

void ns_GINS::LooseCombination::mech() {
    double dt = imuData_[2].t - imuData_[1].t;
    // 先补偿数据，然后机械编排算法更新
    imuData_[2].compensate(imuError_,dt);
    PureIns::updateSinEpoch(imuData_[0],imuData_[1],imuData_[2],insState_[0],insState_[1],insState_[2]);
    // 传递IMU误差,将IMU误差设为本历元的IMU误差，便于下一次IMU数据补偿
    insState_[2].m_error = insState_[1].m_error;
    imuError_ = insState_[2].m_error;
}

int ns_GINS::LooseCombination::ifUpdate(GPST updateTime) {
    GPST lastT = imuData_[1].t, curT = imuData_[2].t;
    if(abs(lastT - updateTime) <= TIME_ALIGN_ERROR){
        return -1;
    }
    else if(abs(curT - updateTime) <=TIME_ALIGN_ERROR){
        return 0;
    }
    else if(lastT - updateTime < 0 && updateTime - curT < 0){
        return 1;
    }
    else return 2;
}

GPST ns_GINS::LooseCombination::getTime() {
    return imuData_[2].t;
}

Matrix ns_GINS::LooseCombination::getStateVariance() {
    return filter_.getMStateVarianceK();
}

void ns_GINS::LooseCombination::printConfig() {
    this->options_.print();
}
