#ifndef _EXPERIMENTMANAGER_
  
// Defines _ANIMALS_ if above
// conditions fails
#define _EXPERIMENTMANAGER_
#include "manifold.h"
#include "cloud_loader.h"
#include "monte_carlo.h"
#include <random>



// int p[10]={};

// for (int i=0; i<nrolls; ++i) {
//   double number = distribution(generator);
//   if ((number>=0.0)&&(number<10.0)) ++p[int(number)];
// }

class ExperimentManager{
public:
	ExperimentManager(float x_low,float x_high,float y_low,float y_high,float z_low,float z_high,std::string map_output_filename, int cluster_count,int feature_count_resolution, int map_count,int x_resolution, int y_resolution,int z_resolution,bool load_map, std::string map_name, bool generate_map_only, bool generate_direction_and_uncertainty){
		this->x_limit[0]=x_low;
		this->x_limit[1]=x_high;
		this->y_limit[0]=y_low;
		this->y_limit[1]=y_high;
		this->z_limit[0]=z_low;
		this->z_limit[1]=z_high;
		this->number_of_clusters=cluster_count;
		this->generate_direction_and_uncertainty=generate_direction_and_uncertainty;
		CloudLoader loader;
		std::cout<<"load map "<<load_map<<std::endl;
		if (load_map){
			std::cout<<"map name "<<map_name<<std::endl;
			loader.ImportFromXyzFile(map_name,1,true,false,",");
			print_string("point cloud size");
			print_float(loader.get_pointcloud().size());			
		}

		for (int i=0;i<map_count;i++){


			if (load_map){
				for (Eigen::Vector3f point:loader.get_pointcloud()){
					this->collection.push_back(point);
				}
			}else{
				std::cout<<"generating random map map "<<std::endl;
				this->map.open("../../Map/"+std::to_string(i)+"_"+map_output_filename);
				if(this->generate_direction_and_uncertainty){
					this->map_dir.open("../../Map/"+std::to_string(i)+"_dir_"+map_output_filename);
				}
				this->generate_random_map();
			}

			if (generate_map_only){
				std::cout<<"generating random map only "<<generate_map_only<<std::endl;
				continue;
			}
			if (feature_count_resolution>10){
				feature_count_resolution=10;
			}



			for (int j=1;j<=feature_count_resolution;j++){
			//for (int j=feature_count_resolution;j>0;j--){
				std::string map_path="../../Map/"+std::to_string(i)+"_"+std::to_string(j)+"_"+map_output_filename;
				this->sub_sampled_map.open(map_path);
				int down_sample_threshold=j*250;
				this->sub_sampled_collection.clear();
				int count=0;
				for (Eigen::Vector3f point:this->collection){
					std::cout<<count<<std::endl;
					if (count%2500<=down_sample_threshold){
						this->sub_sampled_collection.push_back(point);
						this->sub_sampled_map<<std::to_string(point[0])<<","<<std::to_string(point[1])<<","<<std::to_string(point[2])<<std::endl;
					}
					count+=1;
				}
				this->sub_sampled_map.close();
				const auto p1 = std::chrono::system_clock::now();
				int time_since_epoc=std::chrono::duration_cast<std::chrono::seconds>(
			                   p1.time_since_epoch()).count();
				std::string prefix=std::to_string(time_since_epoc)+"_map_"+std::to_string(i)+"_"+std::to_string(j)+"_";
				MonteCarloRun run(x_resolution,y_resolution,z_resolution,x_low,x_high,y_low,y_high,z_low,z_high,prefix,map_path);
				run.run_monte_carlo_optimizer();

				//count+=1;
			}
		}
	}
	//generate N clasters map
	//add features to clusters

	std::vector<Eigen::Vector3f> get_collection(){
		return this->collection;

	}

	void generate_random_map(){
		this->collection.clear();
		int number_of_clusters=this->number_of_clusters;
		generate_cluster_locations(number_of_clusters);
		for (Eigen::Vector3f location: this->cluster_location){
			generate_random_cluster(location[0],location[1],location[2],2500);
		}

	}

	void generate_cluster_locations(int numbers){
		for (int i =0;i<numbers;i++){
			this->cluster_location.push_back(this->generate_cluster_location());
		}
	}

	Eigen::Vector3f generate_cluster_location(){
		float cluster_location_x=this->generate_uniform_number(x_limit[0],x_limit[1]);
		float cluster_location_y=this->generate_uniform_number(y_limit[0],y_limit[1]);
		float cluster_location_z=this->generate_uniform_number(z_limit[0],z_limit[1]);
		Eigen::Vector3f v;
		v<<cluster_location_x,cluster_location_y,cluster_location_z;
		return v;
	}

	float generate_uniform_number(float low, float high){
		std::random_device rd;
		std::default_random_engine generator(rd());
		//std::normal_distribution<double> distribution_x(x_mean,x_std);
		std::uniform_int_distribution<int> distribution_uniform(low,high);
		return distribution_uniform(generator);
	}


	void generate_random_cluster(float x_mean,float y_mean,float z_mean,int sample_count){
		// float x_std=generate_uniform_number(1,(x_limit[1]-x_limit[0])/3);
		// float y_std=generate_uniform_number(1,(y_limit[1]-y_limit[0])/3);
		// float z_std=generate_uniform_number(1,(z_limit[1]-z_limit[0])/3);

		float x_std=generate_uniform_number(15,30);
		float y_std=generate_uniform_number(15,30);
		float z_std=generate_uniform_number(15,30);

		float cluster_uncertainty=generate_uniform_number(1,80);

		float x_dir=generate_uniform_number(-30,30);
		float y_dir=generate_uniform_number(-30,30);
		float z_dir=0.0;		

		Eigen::Vector3f dir;
		dir<<x_dir,y_dir,z_dir;
		dir=dir.normalized();



		std::random_device rd;
		std::default_random_engine generator(rd());
		std::normal_distribution<double> distribution_x(x_mean,x_std);
		std::normal_distribution<double> distribution_y(y_mean,y_std);
		std::normal_distribution<double> distribution_z(z_mean,z_std);		
		for (int i=0; i<sample_count; ++i){
			double x = distribution_x(generator);
			double y= distribution_y(generator);
			double z = distribution_z(generator);
			if  (x<x_limit[0] || x>x_limit[1] ||y<y_limit[0] || y>y_limit[1] ||z<z_limit[0] || z>z_limit[1]){
				i-=1;
				continue;
			}
			Eigen::Vector3f v;
			v<<x,y,z;
			this->collection.push_back(v);
			this->map<<std::to_string(x)<<","<<std::to_string(y)<<","<<std::to_string(z)<<std::endl;
			if(this->generate_direction_and_uncertainty){
				this->map_dir<<std::to_string(dir(0,0))<<","<<std::to_string(dir(1,0))<<","<<std::to_string(dir(2,0))<<","<<cluster_uncertainty<<std::endl;
			}
		}
	}

private:
	std::vector<Eigen::Vector3f> collection;
	std::vector<Eigen::Vector3f> sub_sampled_collection;
	float x_limit[2];
	float y_limit[2];
	float z_limit[2];
	std::ofstream map;
	std::ofstream sub_sampled_map;
	std::ofstream map_dir;
	std::vector<Eigen::Vector3f> cluster_location;

	int number_of_clusters;
	bool generate_direction_and_uncertainty;


};






#endif //_EXPERIMENTMANAGER_