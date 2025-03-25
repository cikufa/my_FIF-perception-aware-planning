#ifndef _MANIFOLD_
#define _MANIFOLD_

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <time.h> 
#include <vector>
#include <typeinfo>
#include <chrono>

#include <fstream> 
#include <sstream> 
#include "cloud_loader.h"


int print_eigen(Eigen::Matrix3f m)
{
    // Eigen Matrices do have rule to print them with std::cout
    std::cout << m << std::endl;
    return 0;
}

int print_eigen_v(Eigen::Vector3f m)
{
    // Eigen Matrices do have rule to print them with std::cout
    std::cout << m << std::endl;
    return 0;
}

int print_string (std::string s)
{
    std::cout << s << std::endl;
    return 0;	
}

int print_float (float s){
    std::cout << s << std::endl;
    return 0;		
}

class FovOptimizerOnManifold{
public:
	
		Eigen::Matrix3f skew(Eigen::Vector3f x){
			Eigen::Matrix3f skew;
			skew << 0, -x[2], x[1],
			     x[2], 0, -x[0],
			     -x[1], x[0], 0;
	        //print_eigen(skew)
	        return skew;
	    }
		
		Eigen::Matrix3f exp_map(Eigen::Vector3f phi){
			// print_string("exp_map<1>");
			Eigen::Matrix3f skew_phi_matrix=this->skew(phi);
			Eigen::Matrix3f I=Eigen::Matrix<float, 3, 3>::Identity();
			// print_eigen(I);
			float phi_norm =phi.norm();
			float sin_phi_norm_div_phi_norm=sin(phi_norm)/phi_norm;	
			// print_string("exp_map<1.5>");
			float one_minus_cos_phi_norm_div_phi_norm_square=(1.0-cos(phi_norm))/(phi_norm*phi_norm);
			// print_string("exp_map<1.6>");
			Eigen::Matrix3f skew_phi_matrix_square=skew_phi_matrix*skew_phi_matrix;
			
		
			Eigen::Matrix3f exp_map=I+sin_phi_norm_div_phi_norm*skew_phi_matrix+one_minus_cos_phi_norm_div_phi_norm_square*skew_phi_matrix_square;
			// print_string("exp_map<2>");
			return exp_map;
	    }	
	    
/*ADDED:*/	
		// FovOptimizerOnManifold(std::string filename_, double visibility_ ,double visibility_angle_,bool importing_, std::vector<Eigen::Vector3f> points_list_, Eigen::Vector3f starting_c_,bool print)
	    FovOptimizerOnManifold(std::string filename_, double visibility_ ,double visibility_angle_,bool importing_, std::vector<Eigen::Vector3f> points_list_, Eigen::Vector3f starting_c_,bool print,Eigen::Vector3f ref_point)
		{
		
	    this->brute_force_max_feature_in_FOV=0;
	   	this->print_path=print;
		this->pathfile.open ("src/rpg_information_field/act_map/FOVData/path.csv");
	    this->pointsfile.open ("src/rpg_information_field/act_map/FOVData/points.csv");
	    this->brute_force_file.open("src/rpg_information_field/act_map/FOVData/brute_force_xyz_indexes_two_degree.csv");
		this->quiversforonepoint.open("src/rpg_information_field/act_map/FOVData/quiversforonepoint.csv", std::ios::app );
		// this->quiversforonepoint.open("/rpg_")
		
// 		##           data generation
// 		r = R.from_euler('z', 0, degrees=True)
		
	    this->R= Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitX())
			  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitY())
			  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitZ());
		//print_eigen(this->R);
		/*ADDED:*/
		
		this->ref_point=ref_point;
  		this->c=starting_c_/starting_c_.norm();
		this->rotated_vec=this->R*this->c;  /*0 * starting_c*/
 		if (importing_==true){
// 			self.points_list=points_list
 			this->points_list=points_list_;
 		
 		}
 		else{
	    	for (int i=0;i<200;i++){
	    		Eigen::Vector3f a;
	    		Eigen::Vector3f b;
	    		a<<rand() % 10+1+20,rand() % 20+1+50,rand() % 20+1+0;
	    		b<<rand() % 10+1-30,rand() % 10+1+10,rand() %50+1+50;
	    		//print_eigen_v(a);
	    		//print_eigen_v(b);
	    		this->points_list.push_back(a); //thisrand() % 10+1
	    		this->points_list.push_back(b);
	    		
	    	} 
	    }
// 		self.t=np.matrix([0.0,0.0,0.0])
	    this->t<<0,0,0;
	    for(Eigen::Vector3f point: this->points_list){

	    	Eigen::Vector3f K__=point-this->t;
	    	K__=K__/K__.norm();	
	    	this->K_list.push_back(K__);
    		std::ostringstream ss;
			ss << K__[0]<<","<< K__[1]<<","<< K__[2]<<std::endl;
			std::string s(ss.str());
    		this->pointsfile<<s;
	    }

	    this->filename=filename_;
	    this->visibility_angle=visibility_angle_;
	    this->optimize_visibility_sigmoid=visibility_;
	    this->optimize_visibility=!visibility_;

	   	this->loader=new CloudLoader;
		/*NOTE: I commented*/
	   	// print_string("loading");
		this->loader->ImportFromXyzFile("src/rpg_information_field/act_map/brute_force_xyz_indexes_two_degree.csv",1,true,false,",");
		// print_string("loaded");
		// std::cout<<"size of data "<<this->loader->get_pointcloud().size()<<std::endl;

	    }	

	   Eigen::Matrix3f get_R(void){
	   		return this->R;
	   }

	   Eigen::Vector3f get_brute_force_best_vector(void){
	   		print_string("get_brute_force_vector");
	   		print_eigen_v(this->brute_force_best_vector);
	   		return this->brute_force_best_vector;
	   }

	   int get_brute_force_max_feature(){

	   	return this->brute_force_max_feature_in_FOV;
	   }

	   int get_optimized_max_feature(){

	   	return this->optimized_max_feature_in_FOV;
	   }
	  
	   void brute_force_search(void){
	   		int optimized_max_feature_in_FOV_=0;
	   		Eigen::Vector3f optimized_FOV_vector=this->R*this->c;
	   		// print_string("optimized_FOV_vector");
	   		// print_eigen_v(optimized_FOV_vector);
			for(Eigen::Vector3f K_: this->K_list){

				if( abs(acos((K_.transpose())*optimized_FOV_vector))<this->visibility_alpha){
					optimized_max_feature_in_FOV_+=1;
				}
			}
			std::cout<<"optimized_max_feature_in_FOV "<<optimized_max_feature_in_FOV_<<std::endl;
			this->optimized_max_feature_in_FOV=optimized_max_feature_in_FOV_;
			int count=0;

			for (Eigen::Vector3f direction:this->loader->get_pointcloud()){
				//std::cout<<"count"<<count;
				count+=1;
				direction=direction/direction.norm();
				 //print_string("direction is");
				 //print_eigen_v(direction);
				Eigen::Vector3f Ideal;
				Ideal<<0.753056,0.657866,0.0108853;
				Ideal=Ideal/Ideal.norm();
				int max_feature_in_FOV=0;
				for(Eigen::Vector3f K_: this->K_list){

					if( abs(acos((K_.transpose())*direction))<this->visibility_alpha){
						max_feature_in_FOV+=1;
					}
				}
				//std::cout<<"max_feature_in_FOV "<<max_feature_in_FOV<<std::endl;
				if (max_feature_in_FOV>this->brute_force_max_feature_in_FOV){
					this->brute_force_max_feature_in_FOV=max_feature_in_FOV;
					this->brute_force_best_vector=direction;
				}
			}
			std::cout<<"brute_force_max_feature_in_FOV "<<this->brute_force_max_feature_in_FOV<<std::endl;
			return;

			this->c=this->brute_force_best_vector;
		    this->R= Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitX())
				  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitY())
				  * Eigen::AngleAxisf(0.0*M_PI, Eigen::Vector3f::UnitZ());
			/*NOTE:*/ 
			// this->optimize();

			Eigen::Vector3f best_updated_brute_force_vector=this->R*this->c;
			int new_max_feature_in_FOV=0;
			for(Eigen::Vector3f K_: this->K_list){

				if( abs(acos((K_.transpose())*best_updated_brute_force_vector))<this->visibility_alpha){
					new_max_feature_in_FOV+=1;
				}
			}

			if (new_max_feature_in_FOV>=this->brute_force_max_feature_in_FOV){
				this->brute_force_max_feature_in_FOV=new_max_feature_in_FOV;
				this->brute_force_best_vector=best_updated_brute_force_vector;
				std::cout<<"updated brute_force_max_feature_in_FOV "<<this->brute_force_max_feature_in_FOV<<std::endl;
			}

	   		print_string("degree_difference");
	   		// print_eigen_v(this->brute_force_best_vector);
	   		print_float(acos(optimized_FOV_vector.transpose()*this->brute_force_best_vector)*180.0/M_PI);

	   }

	   Eigen::Vector3f get_Jacobian_from_K_and_C(float K1,float K2,float K3,float C1,float C2,float C3){
	   			Eigen::Vector3f J;
	   			float a=C2*K3-C3*K2;
	   			float b=C3*K1-C1*K3;
	   			float c=C1*K2-C2*K1;
	   			J<<a,b,c;
	   			return J;	   	
	   }

	   void optimize(){
	   	for (int i=0;i<this->max_iteration;i++){
	   		//std::cout<<"i is "<<i<<std::endl;
	   		Eigen::Vector3f aJ_l;
	   		aJ_l<<0,0,0;
	   		float residual=0.0;
	   		
	   		for(Eigen::Vector3f K_: this->K_list){   			
	   			Eigen::Vector3f K=(K_.transpose())*this->R; //# this -R
	   			float C1=this->c[0];  //#this->c
	   			float C2=this->c[1];
	   			float C3=this->c[2];
	   			float K1=K[0];
	   			float K2=K[1];
	   			float K3=K[2];

	   			Eigen::Vector3f F_Jacobian=this->get_Jacobian_from_K_and_C(K1,K2,K3,C1,C2,C3); //jacobian function
	   			Eigen::Vector3f J=F_Jacobian;
	   			
	   			if (this->optimize_visibility==true){
	   				float KTRC=K_.transpose()*this->R*this->c;
	   				residual=residual+KTRC;
	   				F_Jacobian=2*.35*KTRC*J+.11*J;
	   			}
	   			if (this->optimize_visibility_sigmoid==true){   //optimize visibility sigmoid
	   				float u,KTRC;
	   				u=(K_.transpose())*(this->R)*this->c;
	   				KTRC=u;
	   				residual=residual+KTRC;
					float visibility_alpha;
					float interation_count=(float)i;
					if(interation_count/this->max_iteration<0.1){
						 visibility_alpha=this->visibility_alpha_180;
					}else if(interation_count/this->max_iteration>=0.1 && interation_count/this->max_iteration<0.2){
						 visibility_alpha=this->visibility_alpha_90;
					}else if(interation_count/this->max_iteration>=0.2 && interation_count/this->max_iteration<0.3){
						 visibility_alpha=this->visibility_alpha_45;
					}else if(interation_count/this->max_iteration>=0.3 && interation_count/this->max_iteration<0.4){
						 visibility_alpha=this->visibility_alpha_22_5;
					}else{
						 visibility_alpha=this->visibility_alpha;
					}
	   				float w=(-1.0)*this->ks*(u-cos(this->visibility_alpha)); //visibility_alpha, this->ks
	   				float v=exp(w);
	   				float coeff=(-1.0)*(pow((1+v),(-2))*v*(0.0-this->ks));
	   				F_Jacobian=coeff*J;
				}
		   		
				Eigen::Vector3f aJ=F_Jacobian;
				aJ_l=aJ_l+aJ;
			}
// 						
			if(true){
				//print_string("residual");
				//print_float(residual);

				// float step=(1/(aJ_l.transpose()*aJ_l))*residual;
				//step=.0001*(1.0/(this->K_list.size()/2500.0));
				// step=(1/(this->K_list.size()*M_PI*8));
				/*ADDED: *100*/
				float step=(1/(this->K_list.size()*M_PI*8))*3;
				std::cout<<"step   "<<step<<std::endl;

				// print_float(step);
				aJ_l=step*aJ_l;
				
			}
// 			
			/*ADDED: */  
			this->quiversforonepoint<< std::to_string(this->ref_point[0])<<","<< std::to_string(this->ref_point[1])<<","<< std::to_string(this->ref_point[2])<<","<< std::to_string(this->rotated_vec[0])<<","<< std::to_string(this->rotated_vec[1])<<","<<std::to_string(this->rotated_vec[2])<<","<<i<<std::endl;	
			this->R=this->exp_map(aJ_l)*this->R;  /*NOTE:formula 13*/
			std::cout<<"rotated vec 356    "<<this->R<<"     "<<this->c<<"     "<<this->rotated_vec<<std::endl;
			this->rotated_vec=this->R*this->c;

//> 			#self.ax.scatter3D([self.rotated_vec[0,0]],[self.rotated_vec[1,0]],[self.rotated_vec[2,0]],'blue')
//> 			#self.ax.scatter3D([self.R[0,2]],[self.R[1,2]],[self.R[2,2]],'blue')
			if (this->print_path==true){
				this->pathfile<<std::to_string(rotated_vec[0])<<","<<std::to_string(rotated_vec[1])<<","<<std::to_string(rotated_vec[2])<<std::endl;
			}
// 			self.plotR1.append(rotated_vec[0,0])
// 			self.plotR2.append(rotated_vec[1,0])
// 			self.plotR3.append(rotated_vec[2,0])
			this->plotR1.push_back(rotated_vec[0]);
			this->plotR2.push_back(rotated_vec[1]);
			this->plotR3.push_back(rotated_vec[2]);
// 			sum_result=0
// 			for K__ in self.K_list:
// 				sum_result+=K__*self.R*self.c
			/*
			float sum_result=0;
			for (Eigen::Vector3f K__:this->K_list){
				sum_result+=K__.transpose()*this->R*this->c;
			}
// 			self.plot_sum_x.append(i)
// 			self.plot_sum_y.append(sum_result[0,0])
			this->plot_sum_x.push_back(float(i));
			this->plot_sum_y.push_back(sum_result);
// 			self.plotstep.append(step)
//> 			self.ax.quiver(0, 0, 0, rotated_vec[0,0], rotated_vec[1,0], rotated_vec[2,0], length=1, normalize=True)
			*/
		/*ADDED: degfault .01*/
			if(aJ_l.norm()<.01){
				this->pointsfile.close();
				this->pathfile.close();
				std::cout<<"break cause aJ_l.norm()<..";
				break;
			}
// 			if np.linalg.norm(np.matrix(aJ_l))<.01:
// 				break			
		}
	}

private:
		int max_iteration=30;
		Eigen::Matrix3f R;
		Eigen::Vector3f c;
		Eigen::Vector3f ref_point;
		std::vector<Eigen::Vector3f> points_list;
		Eigen::Vector3f t;
		std::vector<Eigen::Vector3f> K_list;
		std::vector<double> plotR1;
		std::vector<double> plotR2;
		std::vector<double> plotR3;
		std::vector<double> plot_sum_x;
		std::vector<double> plot_sum_y;	

		bool optimize_visibility=false;
		bool optimize_visibility_sigmoid;
		std::string filename;
		double ks=15;
		float visibility_angle=15;
		double visibility_alpha=visibility_angle*M_PI/180.0;
		double visibility_alpha_180=179.0*M_PI/180.0;
		double visibility_alpha_90=90.0*M_PI/180.0;
		double visibility_alpha_45=45.0*M_PI/180.0;
		double visibility_alpha_22_5=22.5*M_PI/180.0;

		std::ofstream quiversforonepoint;
		std::vector<double> plotstep;
		Eigen::Vector3f rotated_vec;
		std::ofstream pointsfile;
		std::ofstream pathfile;
		std::ifstream brute_force_file;
		bool print_path;
		CloudLoader *loader;
		//Eigen::Vector3f starting_c<<1,0,0;
		int brute_force_max_feature_in_FOV;
		int optimized_max_feature_in_FOV;
		Eigen::Vector3f brute_force_best_vector;

};

#endif // _MANIFOLD_