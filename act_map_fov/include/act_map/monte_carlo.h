#ifndef _MONTECARLO_
  
// Defines _ANIMALS_ if above
// conditions fails
#define _MONTECARLO_

#include "manifold.h"
#include "cloud_loader.h"


// 	def plot_set_B(self):
// 		for point in self.icp.set_B_and_Noise:
// 			self.ax1.scatter(point[0],point[1],point[2],c="blue",s=.25)
// 		self.ax1.view_init(elev=89., azim=150)
// 		self.ax1.set_xlabel("x")
// 		self.ax1.set_ylabel("y")
// 		self.ax1.set_title(" python,756x2 ft pts, average_elapsed_time per loc,"+str(1000*self.avg_time)+"(ms)")
// 		self.fig1.savefig("monte_carlo_set_B_0_noise.pdf",dpi=150)
// 		#plt.show()

// class monte_carlo(object):
class MonteCarloRun{
public:
	MonteCarloRun(int x_resolution,int y_resolution,int z_resolution,float x_low,float x_high,float y_low,float y_high,float z_low,float z_high,std::string prefix,std::string mapfilename){
 		this->x_resolution=x_resolution;
 		this->y_resolution=y_resolution;
 		this->z_resolution=z_resolution;
 		this->x_limit[0]=x_low;
 		this->x_limit[1]=x_high;
 		this->y_limit[0]=y_low;
 		this->y_limit[1]=y_high;
 		this->z_limit[0]=z_low;
 		this->z_limit[1]=z_high;


/*NOTE: i commented */
		// this->loader=new CloudLoader;
		// this->loader->ImportFromXyzFile(mapfilename,1,true,false,",");
		// std::cout<<"monte carlo, size of data "<<this->loader->get_pointcloud().size()<<std::endl;


		this->optimizer_monte_carlo_total_time_us=0;
		this->optimizer_monte_carlo_average_time_us=0;
		this->brute_force_search_total_time_us=0;
		this->brute_force_search_average_time_us=0;

		this->quiversfile.open("rpg_information_field/act_map/FOVData/single_run_rotated_quivers.csv");
		this->brute_force_quiversfile.open("rpg_information_field/act_map/FOVData/single_run_brute_force_rotated_quivers.csv");
		this->montecarlopointsfile.open("rpg_information_field/act_map/FOVData/montecarlo_points.csv");

		this->optimizer_avg_time_file.open("rpg_information_field/act_map/FOVData/optimizer_avg_time_file.csv");
		this->brute_force_avg_time_file.open("rpg_information_field/act_map/FOVData/brute_force_avg_time_file.csv");
		this->optimizer_accuracy_file.open("rpg_information_field/act_map/FOVData/optimizer_accuracy_file.csv");
		this->mean.open("rpg_information_field/act_map/FOVData/mean.csv" , std::ios::app);
		this->pointslistfile.open("../../Data/pointslistfile.csv" , std::ios::app);
		this->test.open("rpg_information_field/act_map/FOVData/startingc.csv" , std::ios::app);
		
		for (Eigen::Vector3f point: this->loader->get_pointcloud()){
			this->montecarlopointsfile<<point[0]<<","<<point[1]<<","<<point[2]<<","<<std::endl;
		}

		this->populate_indexes();

	}
	
	void populate_indexes(void){
		for (int i=0; i<this->x_resolution;i++){
			this->x_index.push_back(this->x_limit[0]+(this->x_limit[1]-this->x_limit[0])*i/(float)this->x_resolution);

		}
		for (int i=0; i<this->y_resolution;i++){
			this->y_index.push_back(this->y_limit[0]+(this->y_limit[1]-this->y_limit[0])*i/(float)this->y_resolution);			
		}
		for (int i=0; i<this->z_resolution;i++){
			this->z_index.push_back(this->z_limit[0]+(this->z_limit[1]-this->z_limit[0])*i/(float)this->z_resolution);			
		}
	}

	void populate_local_indexes(Eigen::Vector3f ref_point){
		Eigen::Vector3f pos=ref_point;
		// print_string("ref_point");
		// print_eigen_v(ref_point);
		this->points_list.clear();
		for (Eigen::Vector3f point: this->loader->get_pointcloud()){
			// print_string("point");
			// print_eigen_v(point);
			Eigen::Vector3f point_pos=point-pos;
			// print_string("point_pos");
			// print_eigen_v(point_pos);
			point_pos=point_pos/point_pos.norm();
			// print_string("point_pos");
			// print_eigen_v(point_pos);
			this->points_list.push_back(point_pos);
		}
	}

 /*ADDED:*/
		std::pair<Eigen::Vector3f, Eigen::Vector3f> calculate_pointcloud_mean() {
		Eigen::Vector3f total(0, 0, 0);
		for (const Eigen::Vector3f& point : this->loader->get_pointcloud()) {
			total += point;
		}	
		Eigen::Vector3f totalme = total / static_cast<float>(this->loader->get_pointcloud().size());
		Eigen::Vector3f totalchen = total / total.norm();
		return std::make_pair(totalme, totalchen);
	}
	
	void run_monte_carlo_optimizer(void){
		for (int i=0; i<this->x_resolution;i++){
			for (int j=0; j<this->y_resolution;j++){
				for (int k=0; k<this->z_resolution;k++){
					std::cout<<"----------------------------------------"<<std::endl;
					std::cout<<i<<","<<j<<","<<k<<std::endl;
					// print_string("<0.0>");
					Eigen::Vector3f ref_point;
					// print_string("<0.1>");
					std::cout<<this->x_index[i],this->y_index[j],this->z_index[k];
					// print_string("<0.15>");    
					ref_point<<this->x_index[i],this->y_index[j],this->z_index[k];	
					this->populate_local_indexes(ref_point);
					Eigen::Vector3f starting_c,origin;
					origin<<0,0,0;
					Eigen::Vector3f mean_center_of_all_pointsme, mean_center_of_all_pointsch;
					std::tie(mean_center_of_all_pointsme, mean_center_of_all_pointsch) = this->calculate_pointcloud_mean();
					// std::cout << "mean: " << mean_center_of_all_pointsme << ", " << mean_center_of_all_pointsch << std::endl;
					this->mean<< std::to_string(mean_center_of_all_pointsme[0])<<","<< std::to_string(mean_center_of_all_pointsme[1])<<","<<std::to_string(mean_center_of_all_pointsme[2])<<","<<std::to_string(mean_center_of_all_pointsch[0])<<","<< std::to_string(mean_center_of_all_pointsch[1])<<","<< std::to_string(mean_center_of_all_pointsch[2])<<std::endl;
					
					
					starting_c=mean_center_of_all_pointsme-ref_point;

					/*TODO: TODO:  rabdom starting c*/


					this->manifold=new FovOptimizerOnManifold("FOV_30degree.pdf",true,15.0,true,this->points_list,starting_c,true, ref_point);
					std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
					
					// this->manifold->optimize();
					/*ADDED:*/
					this->manifold->optimize();

					
					/*NOTE: commented*/
					// // print_string("<2>");
					// std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
					// std::cout << "Optimizer Time Difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;	
					// int time_us=std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
					// this->optimizer_monte_carlo_total_time_us+=(float)time_us;
					// this->optimizer_avg_time_file<<std::to_string((float)time_us)<<std::endl;
					Eigen::Vector3f quiver_head=(this->manifold->get_R())*starting_c;

					// begin = std::chrono::steady_clock::now();
					// this->manifold->brute_force_search();
					//print_string("<2>");
					// end = std::chrono::steady_clock::now();
					// std::cout << "Brute Force Time Difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;	
					// time_us=std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
					// this->brute_force_search_total_time_us+=(float)time_us;

					
					// this->brute_force_avg_time_file<<std::to_string((float)time_us)<<std::endl;


					//print_string("<3>");
// 					quiver_head=self.man.R*np.matrix(starting_c).transpose()
// 					self.ax1.quiver(ref_point[0], ref_point[1], ref_point[2], quiver_head[0,0], quiver_head[1,0], quiver_head[2,0], length=50, normalize=True)
					
					// Eigen::Vector3f brute_force_quiver_head=this->manifold->get_brute_force_best_vector();
					quiver_head=quiver_head/quiver_head.norm();
					// float degree_between =acos(brute_force_quiver_head.transpose()*quiver_head)*180.0/M_PI;
					
					// print_string("degree_between");
					// print_float(degree_between);
					// this->degree_diff_record.push_back(degree_between);
					// this->optimizer_accuracy_file<<std::to_string(degree_between)<<","<<std::to_string(this->manifold->get_brute_force_max_feature())<<","<<std::to_string(this->manifold->get_optimized_max_feature())<<","<<std::endl;
					
					
					this->quiversfile<< std::to_string(ref_point[0])<<","<< std::to_string(ref_point[1])<<","<< std::to_string(ref_point[2])<<","<< std::to_string(quiver_head[0])<<","<< std::to_string(quiver_head[1])<<","<< std::to_string(quiver_head[2])<<std::endl;
					

					// print_string("write_to_bruteforce_quiver");
					// this->brute_force_quiversfile<< std::to_string(ref_point[0])<<","<< std::to_string(ref_point[1])<<","<< std::to_string(ref_point[2])<<","<< std::to_string(brute_force_quiver_head[0])<<","<< std::to_string(brute_force_quiver_head[1])<<","<< std::to_string(brute_force_quiver_head[2])<<std::endl;
					// print_string("finished writting");
					//print_string("<4>");
					delete this->manifold;
				}
			}
		}
		/*NOTE: commented*/

		// this->brute_force_search_average_time_us=this->brute_force_search_total_time_us/(this->x_resolution*this->y_resolution*this->z_resolution);
		// this->optimizer_monte_carlo_average_time_us=this->optimizer_monte_carlo_total_time_us/(this->x_resolution*this->y_resolution*this->z_resolution);
		// std::cout<<"optimizer_monte_carlo_average_time is "<<this->optimizer_monte_carlo_average_time_us<<" [us]"<<std::endl;
		// std::cout<<"brute_force_monte_carlo_average_time is "<<this->brute_force_search_average_time_us<<" [us]"<<std::endl;
		// this->optimizer_avg_time_file<<std::to_string(this->optimizer_monte_carlo_average_time_us)<<std::endl;
		// this->brute_force_avg_time_file<<std::to_string(this->brute_force_search_average_time_us)<<std::endl;



	}

private:
// 	def __init__(self,x_resolution=15,y_resolution=15,z_resolution=1):
// 		#pass
// 		self.x_limit=[-300,300]
// 		self.y_limit=[-300,300]
// 		self.z_limit=[0,10]
// 		self.x_resolution=x_resolution
// 		self.y_resolution=y_resolution
// 		self.z_resolution=z_resolution
// 		self.x_index=[]
// 		self.y_index=[]
// 		self.z_index=[]
 		int x_resolution;
 		int y_resolution;
 		int z_resolution;
 		float x_limit[2];
 		float y_limit[2];
 		float z_limit[2];	
 		std::vector<float> x_index;
 		std::vector<float> y_index;
 		std::vector<float> z_index;

// 		#self.man=Manifold_minimize_angle_among_all_target_points(filename="FOV_30degree_montecarlos.pdf",visibility=True,visibility_angle=15.0,importing=False,points_list=[],plotting=True,starting_c=[0,1,0])
 		FovOptimizerOnManifold *manifold;
// 		self.icp=ICP("KME_planes.xyz","xyz",downsample_factor=100,outlier_rate=.0000001,noise_percent=.5,noise_type="gaussian",sigma=100,max_iteration=30,plotting=False,run=False)
 		CloudLoader *loader;
// 		self.points_list=[]
 		std::vector<Eigen::Vector3f> points_list;
 		float avg_time;
 		float optimizer_monte_carlo_total_time_us; //us
 		float optimizer_monte_carlo_average_time_us; //us
 		float brute_force_search_total_time_us;
  		float brute_force_search_average_time_us;

  		std::vector<float> degree_diff_record;

  		std::string prefix;
		std::ofstream quiversfile;
	    std::ofstream mean;
  		std::ofstream pointslistfile;
		std::ofstream test;
		std::ofstream brute_force_quiversfile;
		std::ofstream montecarlopointsfile;

 		std::ofstream optimizer_avg_time_file;
 		std::ofstream brute_force_avg_time_file;

 		std::ofstream optimizer_accuracy_file;
};






#endif// _MONTECARLO_