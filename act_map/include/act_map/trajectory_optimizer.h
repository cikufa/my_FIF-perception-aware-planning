
#ifndef _TRAJECTORYOPMANIFOLD_

#define _TRAJECTORYOPMANIFOLD_

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





template <typename T> void print(T x)
{

    std::cout<<x<<std::endl;
}




class TrajectoryOptimizerOnManifold{
public:
	TrajectoryOptimizerOnManifold(std::string output_initial_trajectory_filename,std::string input_pointcloud_filename,std::string output_pointcloud_filename,bool use_direction, bool use_uncertainty,std::string input_direction_and_uncertainty_filename,std::string output_pointcloud_dir_filename,std::string input_trajectory_file,std::string output_trajectory_file){

		this->initialization(output_initial_trajectory_filename,input_pointcloud_filename,output_pointcloud_filename,use_direction,use_uncertainty,input_direction_and_uncertainty_filename,output_pointcloud_dir_filename,input_trajectory_file,output_trajectory_file);
	
	};


	TrajectoryOptimizerOnManifold(std::string output_initial_trajectory_filename,std::string output_pointcloud_filename,std::vector<Eigen::Vector3f> pos,std::vector<float> yaw,std::vector<Eigen::Vector3f> pointcloud){

		this->initialization(output_initial_trajectory_filename,output_pointcloud_filename,pos,yaw,pointcloud);
	}

	void initialization(std::string output_initial_trajectory_filename,std::string output_pointcloud_filename,std::vector<Eigen::Vector3f> pos,std::vector<float> yaw, std::vector<Eigen::Vector3f> pointcloud){
		this->output_initial_trajectory_filename=output_initial_trajectory_filename;
		this->initial_trajectory_file.open(this->output_initial_trajectory_filename);

		this->v<<1,1,1;
		this->v=this->v/this->v.norm();
		this->c<<1,0,0;
		this->c=this->c/this->c.norm();
		Eigen::Vector3f starting_position;
		starting_position<<-400,0,0;
		Eigen::Matrix3f starting_rotation;
		starting_rotation<<1,0,0,0,1,0,0,0,1;
		this->montecarlopointsfile.open(output_pointcloud_filename);
		this->trajectory= import_trajectory(pos,yaw);
		//this->loader.ImportFromXyzFile(input_pointcloud_filename,1,true,false,",");




		for (Eigen::Vector3f point:pointcloud){
			this->montecarlopointsfile<<point[0]<<","<<point[1]<<","<<point[2]<<std::endl;
		}

		this->valid_points.clear();

		for (size_t i=0;i<this->loader.get_pointcloud().size();i++){
			this->valid_points.push_back(i);
		}		
	}



	void initialization(std::string output_initial_trajectory_filename,std::string input_pointcloud_filename,std::string output_pointcloud_filename,bool use_direction,bool use_uncertainty,std::string input_direction_and_uncertainty_filename,std::string output_pointcloud_dir_filename,std::string input_trajectory_file_name,std::string output_trajectory_file_name){
		this->use_uncertainty=use_uncertainty;
		this->use_direction=use_direction;
		this->output_initial_trajectory_filename=output_initial_trajectory_filename;
		this->initial_trajectory_file.open(this->output_initial_trajectory_filename);

		this->output_trajectory_file.open(output_trajectory_file_name);

		this->v<<1,1,1;
		this->v=this->v/this->v.norm();
		this->c<<1,0,0;
		this->c=this->c/this->c.norm();
		Eigen::Vector3f starting_position;
		starting_position<<-400,0,0;
		Eigen::Matrix3f starting_rotation;
		starting_rotation<<1,0,0,0,1,0,0,0,1;
		this->montecarlopointsfile.open(output_pointcloud_filename);
		this->montecarlopointsdirfile.open(output_pointcloud_dir_filename);


		this->trajectory=this->loader.ImportFromTrajectoryFile(input_trajectory_file_name,1," ");

		this->loader.ImportFromXyzFile(input_pointcloud_filename,1,true,false," ");
		if(this->use_direction||this->use_uncertainty){
				this->loader.ImportFromDirUncertaintyFile(input_direction_and_uncertainty_filename,1," ");
		}


		//save pointcloud
		for (Eigen::Vector3f point:this->loader.get_pointcloud()){
			this->montecarlopointsfile<<point[0]<<","<<point[1]<<","<<point[2]<<std::endl;
		}


		this->montecarlopointsfile.close();

		//save pointcloud_dir and uncertainty
		std::vector<Eigen::Vector3f> pointcloud_dir_vector=this->loader.get_pointcloud_dir();
		std::vector<float> points_uncertainty_vector=this->loader.get_pointcloud_uncertainty();
		for(int i=0;i<pointcloud_dir_vector.size();i++){
			 this->montecarlopointsdirfile<<pointcloud_dir_vector[i][0]<<","<<pointcloud_dir_vector[i][1]<<","<<pointcloud_dir_vector[i][2]<<","<<points_uncertainty_vector[i]<<std::endl;
		}

		this->montecarlopointsdirfile.close();

		this->valid_points.clear();

		for (size_t i=0;i<this->loader.get_pointcloud().size();i++){
			this->valid_points.push_back(i);
		}
	}


	Eigen::Matrix3f skew(Eigen::Vector3f x){
		Eigen::Matrix3f skew;
		skew << 0, -x[2], x[1],
		     x[2], 0, -x[0],
		     -x[1], x[0], 0;
        return skew;
    }
	Eigen::Matrix3f exp_map(Eigen::Vector3f phi){
		Eigen::Matrix3f skew_phi_matrix=this->skew(phi);
		Eigen::Matrix3f I=Eigen::Matrix<float, 3, 3>::Identity();
		float phi_norm =phi.norm();
		float sin_phi_norm_div_phi_norm=sin(phi_norm)/phi_norm;	
		float one_minus_cos_phi_norm_div_phi_norm_square=(1.0-cos(phi_norm))/(phi_norm*phi_norm);
		Eigen::Matrix3f skew_phi_matrix_square=skew_phi_matrix*skew_phi_matrix;						
		Eigen::Matrix3f exp_map=I+sin_phi_norm_div_phi_norm*skew_phi_matrix+one_minus_cos_phi_norm_div_phi_norm_square*skew_phi_matrix_square;
		return exp_map;
    }


	void populate_local_indexes(Eigen::Vector3f ref_point){
		Eigen::Vector3f pos=ref_point;
		// print_string("ref_point");
		// print_eigen_v(ref_point);
		this->points_list.clear();
		this->points_list_unnormalized.clear();
		std::vector<Eigen::Vector3f> pointcloud_dir;
		std::vector<float> pointcloud_uncertainty;

		if(this->use_direction||this->use_uncertainty){
			this->points_dir_list.clear();
			this->points_uncertainty_list.clear();
			pointcloud_dir=this->loader.get_pointcloud_dir();
			pointcloud_uncertainty=this->loader.get_pointcloud_uncertainty();
		}
		std::vector<Eigen::Vector3f> pointcloud=this->loader.get_pointcloud();
		//for(int i;i<pointcloud.size();i++){

		this->closest=1000000;
		this->furthest=0;
		for (size_t i:this->valid_points){
		//for (Eigen::Vector3f point: this->loader.get_pointcloud()){
			// print_string("point");
			// print_eigen_v(point);
			Eigen::Vector3f point_pos=pointcloud[i]-pos;
			// print_string("point_pos");
			// print_eigen_v(point_pos);
			if (point_pos.norm()>this->furthest)
				this->furthest=point_pos.norm();
			if (point_pos.norm()<this->closest)
				this->closest=point_pos.norm();			
			// print_string("point_pos");
			// print_eigen_v(point_pos);
			
			this->points_list_unnormalized.push_back(point_pos);

			point_pos=point_pos/point_pos.norm();  //candidate to remove
			this->points_list.push_back(point_pos);


			if(this->use_direction||this->use_uncertainty){
				Eigen::Vector3f point_dir=pointcloud_dir[i];
				float point_uncertainty=pointcloud_uncertainty[i];
				if (point_uncertainty>this->max_uncertainty){
					this->max_uncertainty=point_uncertainty;
				}
				this->points_dir_list.push_back(point_dir);
				this->points_uncertainty_list.push_back(point_uncertainty);
			}


		}
	}


   	///velocity finite differencing- sum terms

	std::vector<Eigen::Vector3f> velocity_finite_differencing_jacobian(std::vector<PoseSE3> starting_trajectory,float step_size){
		std::vector<Eigen::Vector3f> trajecotry_jacobian;
	   	//suppose rotation velocity difference is   R2R1^T,  R1 is the earlier rotation, R2 is the later rotation
	   	for (int i =1;i<starting_trajectory.size()-1;i++){ //restric to elements that are not the head or the tail of the vector
	   		//jacobian as the later rotation
	   		Eigen::Vector3f K_2=this->v.transpose();
	   		Eigen::Vector3f C_2=(starting_trajectory[i].get_rotation())*(starting_trajectory[i-1].get_rotation().transpose())*this->v;
	   		Eigen::Vector3f later_J=get_Jacobian_from_K_and_C(K_2[0],K_2[1],K_2[2],C_2[0],C_2[1],C_2[2]);
	   		//jacobian as the earlier rotation
	   		Eigen::Vector3f K_1=(this->v.transpose())*(starting_trajectory[i+1].get_rotation())*(starting_trajectory[i].get_rotation().transpose());
	   		Eigen::Vector3f C_1=this->v;
	   		Eigen::Vector3f early_J=get_Jacobian_of_transposed_exp_from_K_and_C(K_1[0],K_1[1],K_1[2],C_1[0],C_1[1],C_1[2]);
	   		//store into vector

	   		trajecotry_jacobian.push_back(step_size*(later_J+early_J));
	   	}
	   	return trajecotry_jacobian;
   	}

 //   ///acceleration finite differencing - sum terms
	// std::vector<Eigen::Vector3f> acceleration_finite_differencing_jacobian(std::vector<PoseSE3> starting_trajectory){
	// 	Eigen::Vector3f trajecotry_jacobian;
	//    	//suppose rotation velocity difference is   R2R1^T,  R1 is the earlier rotation, R2 is the later rotation
	//    	for (int i =1;i<starting_trajectory.size()-1;i++){ //restric to elements that are not the head or the tail of the vector
	//    		//jacobian as the start
	//    		Eigen::Vector3f K_2=this->v.transpose();
	//    		Eigen::Vector3f C_2=(starting_trajectory[i].get_rotation())*(starting_trajectory[i-1].get_rotation().transpose())*this->v;
	//    		Eigen::Vector3f later_J=get_Jacobian_from_K_and_C(K_2[0],K_2[1],K_2[2],C_2[0],C_2[1],C_2[2]);
	//    		//jacobian as the mid
	//    		Eigen::Vector3f K_1=(this->v.transpose())*(starting_trajectory[i+1].get_rotation())*(starting_trajectory[i].get_rotation().transpose());
	//    		Eigen::Vector3f C_1=this->v;
	//    		Eigen::Vector3f early_J=get_Jacobian_of_transposed_exp_from_K_and_C(K_1[0],K_1[1],K_1[2],C_1[0],C_1[1],C_1[2]);
	//    		//jacobian as the later

	//    		//store into vector
	//    		trajecotry_jacobian.push_back(later_J+early_J);
	//    	}
	//    	return trajecotry_jacobian;
 //   	}
   ///



	Eigen::Vector3f get_Jacobian_from_K_and_C(float K1,float K2,float K3,float C1,float C2,float C3){
		Eigen::Vector3f J;
		float a=C2*K3-C3*K2;
		float b=C3*K1-C1*K3;
		float c=C1*K2-C2*K1;
		J<<a,b,c;
		return J;	   	
   	}
   	Eigen::Vector3f get_Jacobian_of_transposed_exp_from_K_and_C(float K1,float K2,float K3,float C1,float C2,float C3){
		Eigen::Vector3f J;
		float a=C3*K2-C2*K3;
		float b=C1*K3-C3*K1;
		float c=C2*K1-C1*K2;
		J<<a,b,c;
		return J;	   	
   	}	

   	void change_valid_points_list(std::vector<size_t> valid_list){
   		this->valid_points=valid_list;
   	}




   	std::vector<PoseSE3>  import_trajectory(std::vector<Eigen::Vector3f> traj_positions, std::vector<float>  yaws){
   			std::vector<PoseSE3> trajectory;
   			Eigen::Vector3f dummy_vector;

   			this->Matrix_from_RPY_degree(0,0,yaws[0]);



   			dummy_vector=this->c;
   			Eigen::Matrix3f starting_rotation=this->Matrix_from_RPY_degree(0,0,yaws[0]);
   			dummy_vector=starting_rotation*dummy_vector;
   			Eigen::Vector3f starting_position=traj_positions[0];

   			this->initial_trajectory_file<<std::to_string(starting_position[0])<<","<<std::to_string(starting_position[1])<<","<<std::to_string(starting_position[2])<<","<<std::to_string(dummy_vector[0])<<","<<std::to_string(dummy_vector[1])<<","<<std::to_string(dummy_vector[2])<<std::endl;
   			for (int i=0; i<traj_positions.size();i++){
   			//for (Eigen::Vector3f pos:traj_positions){
   				
	   			Eigen::Vector3f v,rotated_v,R_input_rotated_v,current_position;
	   			v=(this->c)*10.0;
	   			Eigen::Matrix3f current_rotation=this->Matrix_from_RPY_degree(0,0,yaws[i]);
	   			rotated_v=current_rotation*v;
	   			current_position=traj_positions[i];
   				this->initial_trajectory_file<<std::to_string(current_position[0])<<","<<std::to_string(current_position[1])<<","<<std::to_string(current_position[2])<<","<<std::to_string(rotated_v[0])<<","<<std::to_string(rotated_v[1])<<","<<std::to_string(rotated_v[2])<<std::endl;
   				PoseSE3 pose(current_position,current_rotation);
   				trajectory.push_back(pose);
   			}

   			return trajectory;

   	}



   	std::vector<PoseSE3>  generate_trajectory_start(bool smooth,float interval_length,int waypoint_count, Eigen::Vector3f starting_position,Eigen::Matrix3f starting_rotation){
   		std::vector<PoseSE3> trajectory;
   		Eigen::Vector3f current_position=starting_position;
   		Eigen::Vector3f dummy_vector;
   		//dummy_vector<<0,1,0;
   		dummy_vector=this->c;
   		dummy_vector=starting_rotation*dummy_vector;
   		this->initial_trajectory_file<<std::to_string(starting_position[0])<<","<<std::to_string(starting_position[1])<<","<<std::to_string(starting_position[2])<<","<<std::to_string(dummy_vector[0])<<","<<std::to_string(dummy_vector[1])<<","<<std::to_string(dummy_vector[2])<<std::endl;
   		PoseSE3 startingpose(starting_position,starting_rotation);
   		trajectory.push_back(startingpose);
   		for (int i=0;i<waypoint_count;i++){
   			Eigen::Matrix3f R=this->Matrix_from_RPY_degree(0,0,-30.0+rand()%60);
   			Eigen::Vector3f v,rotated_v,R_input_rotated_v;
   			v<<1.0*interval_length,0,0;
   			rotated_v=R*v;
   			current_position=current_position+rotated_v;
   			Eigen::Matrix3f R_input=this->Matrix_from_RPY_degree(0,0,-45+rand()%90);


   			R_input_rotated_v=R_input*v;
   			if(smooth){
   				PoseSE3 pose(current_position,R);
   				trajectory.push_back(pose);
   				this->initial_trajectory_file<<std::to_string(current_position[0])<<","<<std::to_string(current_position[1])<<","<<std::to_string(current_position[2])<<","<<std::to_string(rotated_v[0])<<","<<std::to_string(rotated_v[1])<<","<<std::to_string(rotated_v[2])<<std::endl;
   			}else{
   				PoseSE3 pose(current_position,R_input);
   				trajectory.push_back(pose);  
   				this->initial_trajectory_file<<std::to_string(current_position[0])<<","<<std::to_string(current_position[1])<<","<<std::to_string(current_position[2])<<","<<std::to_string(R_input_rotated_v[0])<<","<<std::to_string(R_input_rotated_v[1])<<","<<std::to_string(R_input_rotated_v[2])<<std::endl; 				
   			}
   			
   		}

   		return trajectory;	   			
   	}

   	// std::vector<PoseSE3>  generate_trajectory_start_end(float interval_length,int waypoint_count,){
   	// 	std::vector<PoseSE3> trajectory;
   	// 	Eigen::Vector3f current_position;

		   			
   	// }

   	// std::vector<PoseSE3>  import_trajectory(std::string filename){
   	// 	std::vector<PoseSE3> trajectory;
   	// 	Eigen::Vector3f current_position;

		   			
   	// }


   	Eigen::Matrix3f Matrix_from_RPY_degree(float x,float y,float z){//XYZ order, //degree
	    Eigen::Matrix3f R;
	    R= Eigen::AngleAxisf(x*M_PI/180.0, Eigen::Vector3f::UnitX())
	  	* Eigen::AngleAxisf(y*M_PI/180.0, Eigen::Vector3f::UnitY())
	  	* Eigen::AngleAxisf(z*M_PI/180.0, Eigen::Vector3f::UnitZ());





	  	return R;
	}
   	//import or generate trajectory

	void write_arrow_to_file(Eigen::Vector3f position,Eigen::Vector3f ray_direction){
		this->initial_trajectory_file<<std::to_string(position[0])<<","<<std::to_string(position[1])<<","<<std::to_string(position[2])<<","<<std::to_string(ray_direction[0])<<","<<std::to_string(ray_direction[1])<<","<<std::to_string(ray_direction[2])<<std::endl;
	}

	void write_arrow_to_output_file(Eigen::Vector3f position,Eigen::Vector3f ray_direction){
		this->output_trajectory_file<<std::to_string(position[0])<<","<<std::to_string(position[1])<<","<<std::to_string(position[2])<<","<<std::to_string(ray_direction[0])<<","<<std::to_string(ray_direction[1])<<","<<std::to_string(ray_direction[2])<<std::endl;
	}


  Eigen::Vector3f calculate_FOV_jacobian_for_pose(PoseSE3 pos,int iteration_count){
  	this->populate_local_indexes(pos.get_position());
  	Eigen::Matrix3f R=pos.get_rotation();
 		Eigen::Vector3f aJ_l;
 		aJ_l<<0,0,0;
 		float residual=0.0;
 		//print_string("residual before");
 		//print_float(residual);
// 			for K_ in self.K_list:
 		int counter=0;
 		for(Eigen::Vector3f K_: this->points_list){//#
 			//print_string("start jacobian");
 			//print_eigen_v(K_);
// 				K=K_*self.R
// 				#c=
// 				#print (K)
// 				#print (K[0,1])
// 				C1=self.c[0,0]
// 				C2=self.c[1,0]
// 				C3=self.c[2,0]
// 				K1=K[0,0]
// 				K2=K[0,1]
// 				K3=K[0,2]

 			Eigen::Vector3f K=(K_.transpose())*R; //# this -R
 			// print_string("K");
 			// print_eigen_v(K);
 			// print_string("this->c");
 			// print_eigen_v(this->c);
 			float C1=this->c[0];  //#this->c
 			float C2=this->c[1];
 			float C3=this->c[2];
 			float K1=K[0];
 			float K2=K[1];
 			float K3=K[2];


// 				#print (C1,C2,C3)
// 				#print (K1,K2,K3)
// 				#print("list",[C2*K3-C3*K2,C3*K1-C1*K3,C1*K2-C2*K1])
// 				J=np.matrix([C2*K3-C3*K2,C3*K1-C1*K3,C1*K2-C2*K1]).transpose()
// 				F_Jacobian=J
// 				if self.optimize_visibility==True:
// 					KTRC=K_*self.R*self.c
// 					#holder=
// 					F_Jacobian=2*self.alpha2*KTRC[0,0]*J+self.alpha1*J
// 				if self.optimize_visibility_sigmoid==True:
// 					u=KTRC=K_*self.R*self.c
// 					w=(-1.0)*self.ks*(u-math.cos(self.visibility_alpha))
// 					v=math.exp(w)
// 					coeff=(-1.0)*((1+v)**(-2))*v*(0.0-self.ks)
// 					F_Jacobian=coeff*J

 			Eigen::Vector3f F_Jacobian=this->get_Jacobian_from_K_and_C(K1,K2,K3,C1,C2,C3); //jacobian function
 			Eigen::Vector3f J=F_Jacobian;
 			// print_string("J");
 			// print_eigen_v(J);
 			// print_string("F_Jacobian-j");
 			// print_eigen_v(F_Jacobian);
 			if (this->optimize_visibility_sigmoid==true){   //optimize visibility sigmoid
 				float u,KTRC;
 				u=(K_.transpose())*(R)*this->c;
 				KTRC=u;
 				residual=residual+KTRC;



				float visibility_alpha;
				if(iteration_count/this->max_iteration<0.05){
					 visibility_alpha=this->visibility_alpha_180;
				}else if(iteration_count/this->max_iteration>=0.05 && iteration_count/this->max_iteration<0.1){
					 visibility_alpha=this->visibility_alpha_90;
				}else if(iteration_count/this->max_iteration>=0.1 && iteration_count/this->max_iteration<0.15){
					 visibility_alpha=this->visibility_alpha_60;
				}else{
					 visibility_alpha=this->visibility_alpha;
				}


 				float w=(-1.0)*this->ks*(u-cos(visibility_alpha)); //visibility_alpha, this->ks
 				float v=exp(w);
 				float coeff=(-1.0)*(pow((1+v),(-2))*v*(0.0-this->ks));
 				F_Jacobian=coeff*J;
			}

				// # Nov11 Insertion Point,triangulation direction
				// if self.use_direction==True:
			float alpha=4.0;
			if(this->use_direction){

				// 	direction_dot=np.matrix(K)*np.matrix(self.direction_list[kk]).transpose()
				float direction_dot=(K.transpose()*this->points_dir_list[counter]);
				// 	print("final coeff",1.0-np.abs(direction_dot[0,0]))
				if(this->DEBUG){
					std::cout<<"direction_dot is " <<direction_dot<<std::endl;
				}
				// 	alpha=alpha*(1.0-np.abs(direction_dot[0,0]))
				alpha=alpha*(1.0-fabs(direction_dot));
				F_Jacobian=F_Jacobian*alpha;
			}

			float uncertainty_alpha=3;

			if(this->use_uncertainty){
				 float point_uncertainty=this->points_uncertainty_list[counter];
				 uncertainty_alpha=1.0*point_uncertainty/this->max_uncertainty;

				 F_Jacobian=F_Jacobian*uncertainty_alpha;
			}



			 float distance=this->points_list_unnormalized[counter].norm();
			 float distance_weight=1.0-(distance-this->closest)/(this->furthest-this->closest);
			 F_Jacobian=F_Jacobian*distance_weight;

		
   		// print_string("F_Jacobian F_Jacobian");
   		//std::cout << typeid(F_Jacobian).name() << std::endl;
   		//std::cout<<F_Jacobian<<std::endl;
   		// print_eigen_v(F_Jacobian);

// 				#print (J)
// 				alpha=.001
// 				#alpha=(F_Jacobian.transpose()*F_Jacobian)[0,0]
// 				#print("F_Jacobian.transpose()*F_Jacobian",F_Jacobian.transpose()*F_Jacobian)
// 				#print ("alpha",alpha)
// 				if True:
// 					alpha=1
// 				aJ=alpha*F_Jacobian
		Eigen::Vector3f aJ=F_Jacobian;

// 				aJ_l=[aJ_l[0]+aJ[0,0],aJ_l[1]+aJ[1,0],aJ_l[2]+aJ[2,0]]
		//aj_l<<aJ_l[0]+aJ[0],aJ_l[1]+aJ[1],aJ_l[2]+aJ[2];
		aJ_l=aJ_l+aJ;
		counter++;
		}
		float step_size=(10/(this->points_list.size()*M_PI*2));



		aJ_l=aJ_l*step_size;
		return aJ_l;
  }


	void optimize(bool write_to_file){
		for (int i =0;i<this->max_iteration;i++){
	
			std::vector<Eigen::Vector3f> trajecotry_jacobian=this->velocity_finite_differencing_jacobian(this->trajectory,0.5);

			//file write will write the entire trajectory for the iteration
			// std::cout<<"op <2.1>"<<std::endl;
			if (write_to_file){
				this->initial_trajectory_file<<std::endl;
				Eigen::Matrix3f R=this->trajectory[0].get_rotation();
				Eigen::Vector3f v_;
				//v_<<0,1,0;
				v_=this->c;
				v_=R*v_;
				// std::cout<<"op <2.2>"<<std::endl;
				this->write_arrow_to_file(this->trajectory[0].get_position(),v_);
			}

			// std::cout<<"op <2>"<<std::endl;



			for (int j =0;j<trajecotry_jacobian.size();j++){

				//print("trajecotry_jacobian[j]");
				//print(trajecotry_jacobian[j]);
				// print("trajecotry_jacobian.size()");
				// print(trajecotry_jacobian.size());
				// print("trajectory.size()");
				// print(trajectory.size());
				Eigen::Vector3f FOV_Jacobian,combined_Jacobian;
				FOV_Jacobian=calculate_FOV_jacobian_for_pose(trajectory[j+1],i);
				combined_Jacobian=FOV_Jacobian+trajecotry_jacobian[j];
				combined_Jacobian=FOV_Jacobian;
				Eigen::Matrix3f R=this->exp_map(combined_Jacobian)*(trajectory[j+1].get_rotation());
				//R=this->exp_map(FOV_Jacobian)*(trajectory[j+1].get_rotation()); //overwrite, FOV only
				this->trajectory[j+1].set_rotation(R);
				Eigen::Vector3f v;
				//v<<0,1,0;
				v=this->c;
				v=R*v;
				if (write_to_file){
					this->write_arrow_to_file(this->trajectory[j+1].get_position(),v);
				}	
			}

			// std::cout<<"op <3>"<<std::endl;

			if (write_to_file){
				Eigen::Matrix3f R=this->trajectory.back().get_rotation();
				Eigen::Vector3f v__;
				//v__<<0,1,0;
				v__=this->c;
				v__=R*v__;
				this->write_arrow_to_file(this->trajectory.back().get_position(),v__);
			
			}
			// std::cout<<"op <4>"<<std::endl;

		}

		if (write_to_file){
			for (PoseSE3 pose:this->trajectory){
				Eigen::Matrix3f R=pose.get_rotation();
				std::cout<<"pose rotation"<<R <<std::endl;
				Eigen::Vector3f v__;
				//v__<<0,1,0;
				v__=this->c;
				v__=R*v__;

				std::cout<<"v__"<<v__ <<std::endl;
				this->write_arrow_to_output_file(pose.get_position(),v__);
			}
		}


	}


private:
	Eigen::Vector3f v; //vector v seems to have no importance in theory, only as a reference
	std::vector<PoseSE3> trajectory;
	std::string output_initial_trajectory_filename;
	std::ofstream initial_trajectory_file;
	std::ofstream output_trajectory_file;
	int max_iteration=30;
	CloudLoader loader;
	std::vector<Eigen::Vector3f> points_list;
	std::vector<Eigen::Vector3f> points_list_unnormalized;
	std::vector<Eigen::Vector3f> points_dir_list;
	std::vector<float> points_uncertainty_list;
	double ks=15;
	float visibility_angle=15.0;
	double visibility_alpha=visibility_angle*M_PI/180.0;
	double visibility_alpha_180=179.0*M_PI/180.0;
	double visibility_alpha_90=90.0*M_PI/180.0;
	double visibility_alpha_60=60.0*M_PI/180.0;
	bool optimize_visibility_sigmoid=true;
	Eigen::Vector3f c; //camera principle axis
	std::ofstream montecarlopointsfile;
	std::ofstream montecarlopointsdirfile;

	std::vector<size_t> valid_points;

	bool use_uncertainty=false;
	bool use_direction=false;
	float max_uncertainty=0;
	bool DEBUG=false;

	float closest;
	float furthest;

};





#endif