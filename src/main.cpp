#include <stdio.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
#include "modern_robotics.h"
#include "PropertyDefinition.h"

#include "LieDynamics.h"
#include "PoEKinematics.h"

# define M_PI           3.14159265358979323846 
using namespace std;
using namespace Eigen;
using namespace mr;
using namespace HYUMotionKinematics;
using namespace HYUMotionDynamics;


typedef Matrix<float,ROBOT_DOF,1> Jointf;
typedef struct STATE{
	float q[ROBOT_DOF];
	float q_dot[ROBOT_DOF];
	float q_ddot[ROBOT_DOF];
	float torque[ROBOT_DOF];
	float dq[ROBOT_DOF];
	float dq_dot[ROBOT_DOF];
	float dq_ddot[ROBOT_DOF];
	Jointf j_q;
	
};

void printMatrixList(const std::vector<Eigen::MatrixXd> M){

	std::cout<<"======================List====================="<<std::endl;
	for(int i=0;i<M.size();i++){
		std::cout<<"-----------------------"<<i<<"-----------------------"<<std::endl;
		std::cout<<M.at(i)<<std::endl;
		std::cout<<"\n"<<std::endl;
	}



}

int main(){
	Vector3f w[ROBOT_DOF];
	Vector3f p[ROBOT_DOF];	
	Vector3f L[ROBOT_DOF];		
	Vector3f CoM_URDF[ROBOT_DOF];		
	Vector3f CoM[ROBOT_DOF];	
	Vector3f CoM_[ROBOT_DOF+1];	
	
	Matrix3f inertia[ROBOT_DOF];	
	float mass[ROBOT_DOF];
	HYUMotionKinematics::PoEKinematics pKin = PoEKinematics();
	HYUMotionKinematics::PoEKinematics pCoM = PoEKinematics();
	HYUMotionDynamics::Liedynamics     pDyn = Liedynamics(pKin,pCoM);
	vector<MatrixXd> Mlist;
	vector<MatrixXd> CoM_Mlist;
	vector<MatrixXd> Glist;
	
	
	
	w[0] << 0,0,1;
	w[1] << 0,1,0;	
	w[2] << 0,0,1;
	w[3] << -1,0,0;	
	w[4] << 0,0,1;
	w[5] << 0,1,0;	
	
	p[0]<<BASE_X,BASE_Y,BASE_Z+LINK_01;
	p[1]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12;
	p[2]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23;
	p[3]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34;
	p[4]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45;
	p[5]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56;
	
	L[0]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12;
	L[1]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23;
	L[2]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34;
	L[3]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45;
	L[4]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56;
	L[5]<<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56+LINK_6E;
	
	CoM_URDF[0]<< 0.0 ,0 ,0.049473;
	CoM_URDF[1]<< 0.001501, 0, 0.042572;
	CoM_URDF[2]<< 0.000454 ,0.002601 ,0.084558;
	CoM_URDF[3]<< -0.002133 ,-0.000212 ,0.098828;
	CoM_URDF[4]<< 0.002354, -0.00032, 0.094781;
	CoM_URDF[5]<< 0 ,0.002249, 0.09774;
	CoM_URDF[6]<< 0.000084,-0.001022,0.00897;
	
	
	CoM[0] <<BASE_X+CoM_URDF[1](0),BASE_Y+CoM_URDF[1](1),BASE_Z+LINK_01+CoM_URDF[1](2);
	CoM[1] <<BASE_X+CoM_URDF[2](0),BASE_Y+CoM_URDF[2](1),BASE_Z+LINK_01+LINK_12+CoM_URDF[2](2);
	CoM[2] <<BASE_X+CoM_URDF[3](0),BASE_Y+CoM_URDF[3](1),BASE_Z+LINK_01+LINK_12+LINK_23+CoM_URDF[3](2);
	CoM[3] <<BASE_X+CoM_URDF[4](0),BASE_Y+CoM_URDF[4](1),BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+CoM_URDF[4](2);	
	CoM[4] <<BASE_X+CoM_URDF[5](0),BASE_Y+CoM_URDF[5](1),BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+CoM_URDF[5](2);	
	CoM[5] <<BASE_X+CoM_URDF[6](0),BASE_Y+CoM_URDF[6](1),BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56+CoM_URDF[6](2);	
	
	CoM_[0] = CoM[0];
	CoM_[1] = CoM[1];
	CoM_[2] = CoM[2];
	CoM_[3] = CoM[3];
	CoM_[4] = CoM[4];
	CoM_[5] = CoM[5];
	CoM_[6] <<BASE_X,BASE_Y,BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56+LINK_6E;	
	

	MatrixXd Slist = w_p_to_Slist(w,p,ROBOT_DOF);
	MatrixXd M = MatrixXd::Identity(4,4);
	MatrixXd thetaList(6,1);
	MatrixXd dthetaList(6,1);
	
	M(0,3)=BASE_X;
	M(1,3)=BASE_Y;
	M(2,3)=BASE_Z+LINK_01+LINK_12+LINK_23+LINK_34+LINK_45+LINK_56+LINK_6E;
	thetaList<<0,M_PI/3,0,0,0,0;
	dthetaList<<0,0.1,0,0,0,0;
	
	MatrixXd T = FKinSpace(M,Slist,thetaList);

	cout<<"Forward Kinematics : \n"<<T<<endl;
	Eigen::MatrixXd J= JacobianSpace(Slist,thetaList);
	cout<<"Jacobian Space : \n"<<J<<endl;
	
	inertia[0]<<   J_Ixx_1,J_Ixy_1,J_Ixz_1,
			J_Ixy_1,J_Iyy_1,J_Iyz_1,
			J_Ixz_1,J_Iyz_1,J_Izz_1;
	inertia[1]<<   J_Ixx_2,J_Ixy_2,J_Ixz_2,
			J_Ixy_2,J_Iyy_2,J_Iyz_2,
			J_Ixz_2,J_Iyz_2,J_Izz_2;
	inertia[2]<<   J_Ixx_3,J_Ixy_3,J_Ixz_3,
			J_Ixy_3,J_Iyy_3,J_Iyz_3,
			J_Ixz_3,J_Iyz_3,J_Izz_3;
	inertia[3]<<   J_Ixx_4,J_Ixy_4,J_Ixz_4,
			J_Ixy_4,J_Iyy_4,J_Iyz_4,
			J_Ixz_4,J_Iyz_4,J_Izz_4;
	inertia[4]<<   J_Ixx_5,J_Ixy_5,J_Ixz_5,
			J_Ixy_5,J_Iyy_5,J_Iyz_5,
			J_Ixz_5,J_Iyz_5,J_Izz_5;
	inertia[5]<<   J_Ixx_6,J_Ixy_6,J_Ixz_6,
			J_Ixy_6,J_Iyy_6,J_Iyz_6,
			J_Ixz_6,J_Iyz_6,J_Izz_6;	
			
	mass[0] = MASS_1;
	mass[1] = MASS_2;
	mass[2] = MASS_3;
	mass[3] = MASS_4;
	mass[4] = MASS_5;
	mass[5] = MASS_6;
	
	for(int i =0;i<ROBOT_DOF;i++){
		pDyn.UpdateDynamicInfo(inertia[i],mass[i],i+1);
		pCoM.UpdateKinematicInfo_R(w[i],p[i],CoM[i],i+1);
		pKin.UpdateKinematicInfo_R(w[i],p[i],L[i],i+1);
	}
	float act_q[6]={0.0,M_PI/3,0.0,0,0,0};
	float act_q_dot[6]={0.0,0.1,0.0,0.0,0.0,0.0};
	
   

	CoM_Mlist = getCoM_Mlist(CoM_,ROBOT_DOF);
	Mlist = getMlist(L,p,ROBOT_DOF);
	
	pKin.HTransMatrix(act_q); 
	pDyn.Prepare_Dynamics(act_q,act_q_dot);
	Matrixf Mmat = pDyn.M_Matrix();
	Matrixf Cmat = pDyn.C_Matrix();
	Matrix<float,ROBOT_DOF,1> Gmat = pDyn.G_Matrix();
	


	vector<MatrixXd> HTransList;
	HTransList = getHTranslist(p,M,Slist,thetaList);
	cout<<"===================HTransList================="<<endl;
	printMatrixList(HTransList);
	Glist = getGlist(inertia,mass,ROBOT_DOF);

	printMatrixList(CoM_Mlist);

	VectorXd g(3,1);
	g<<0.0,0.0,-9.8;
	Eigen::MatrixXd Mmat_2 = MassMatrix(thetaList,CoM_Mlist,Glist,Slist);
	Eigen::VectorXd Cmat_2 = VelQuadraticForces(thetaList,dthetaList,CoM_Mlist,Glist,Slist);
	Eigen::VectorXd Gmat_2 = GravityForces(thetaList,g,CoM_Mlist,Glist,Slist);

	cout<<"Gmat : \n"<<Gmat<<endl;

	cout<<"Gmat_2 : \n"<<Gmat_2<<endl;

	return 0;

}
