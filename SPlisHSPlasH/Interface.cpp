#include "Interface.h"
#include <iostream>
#include "Shader.h"
#include "Utils/Utils.h"
#include "SPlisHSPlasH/DFSPH/DFSPH_CUDA.h"
#include <fstream>
#include <sstream>

//first I need a special function that will contain the fluid structure and that will be used to launch all the calls
class SingletonDFSPHCUDA{
public:
    static SPH::DFSPHCUDA* getFluid(bool delete_fluid=false){
        static SPH::DFSPHCUDA* fluid=NULL;

        //that is used to clear
        if(delete_fluid){
            if (fluid!=NULL){
                delete fluid; fluid=NULL;
            }
            return NULL;
        }

        if (fluid==NULL){
            fluid=new SPH::DFSPHCUDA();
        }
        return fluid;


    }

    static SPH::Shader& getShader(int type=0){
        static SPH::Shader m_shader_opaque;
        static SPH::Shader m_shader_translucent;
        static bool first_pass=true;
        if (first_pass){
            first_pass=false;

            //opaque shader
            {
                std::string config_folder_path=get_folder_path("configuration_data",4);

                std::string vertFile = config_folder_path+ "shaders/vs_points.glsl";
                std::string fragFile = config_folder_path+ "shaders/fs_points.glsl";

                m_shader_opaque.compileShaderFile(GL_VERTEX_SHADER, vertFile);
                m_shader_opaque.compileShaderFile(GL_FRAGMENT_SHADER, fragFile);
                m_shader_opaque.createAndLinkProgram();
                m_shader_opaque.begin();
                m_shader_opaque.addUniform("modelview_matrix");
                m_shader_opaque.addUniform("projection_matrix");
                m_shader_opaque.addUniform("radius");
                m_shader_opaque.addUniform("viewport_width");
                m_shader_opaque.addUniform("color");
                m_shader_opaque.addUniform("projection_radius");
                m_shader_opaque.addUniform("max_velocity");
                m_shader_opaque.end();
            }
            //transparent shader

            {
                std::string config_folder_path=get_folder_path("configuration_data",4);

                std::string vertFile = config_folder_path+ "shaders/vs_points.glsl";
                std::string fragFile = config_folder_path+ "shaders/fs_points_transparent.glsl";

                m_shader_translucent.compileShaderFile(GL_VERTEX_SHADER, vertFile);
                m_shader_translucent.compileShaderFile(GL_FRAGMENT_SHADER, fragFile);
                m_shader_translucent.createAndLinkProgram();
                m_shader_translucent.begin();
                m_shader_translucent.addUniform("modelview_matrix");
                m_shader_translucent.addUniform("projection_matrix");
                m_shader_translucent.addUniform("radius");
                m_shader_translucent.addUniform("viewport_width");
                m_shader_translucent.addUniform("color");
                m_shader_translucent.addUniform("projection_radius");
                m_shader_translucent.addUniform("max_velocity");
                m_shader_translucent.end();
            }

        }

        if (type==0){
            return m_shader_opaque;
        }else{
            return m_shader_translucent;
        }
    }
};

void Interface::loadFluid(){
    //for now I'll init it by loading from file
    SingletonDFSPHCUDA::getFluid()->handleSimulationLoad(true,false,true,false,true,false);
    SingletonDFSPHCUDA::getFluid()->reset();


    //also init the shader
    std::cout<<"initialising shader start"<<std::endl;
     SingletonDFSPHCUDA::getShader();
     std::cout<<"initialising shader end"<<std::endl;
}

void Interface::initFluid(double timeStep){
    std::cout<<"Interface::initFluid start"<<std::endl;

     handleDynamicBodiesPause(true);

    //Beause it is impossible o just start the simulation with a large time step immediatly (idk why btw)
    //the solution I'll use will be to do a stabilisaion period of 10 timestepswhen initalising the fluid
    //during those steps I'll start with a 1ms duration and go to the desired duration
    //*
     handleDynamicBodiesPause(true);
     /*/
     int nb_iter_init=10;
    double delta=(timeSrtep-0.001)/nb_iter_init;
    for (int i=0;i<10;++i){
        updateTimeStepDuration(0.001+delta*i);
        fluidSimulationStep();
        zeroFluidVelocities();
    }
    updateTimeStepDuration(timeStep);
    //fluidSimulationStep();
    //*/
    //*/

     handleDynamicBodiesPause(false);


     std::cout<<"Interface::initFluid end"<<std::endl;
}

void Interface::releaseFluid(){
    SingletonDFSPHCUDA::getFluid(true);
}


void Interface::updateDynamicBodies(const std::vector<SPH::DynamicBody> &vect_new_info){
    SingletonDFSPHCUDA::getFluid()->updateRigidBodies(vect_new_info);
}

void Interface::forceUpdateDynamicBodies(){
    SingletonDFSPHCUDA::getFluid()->forceUpdateRigidBodies();
}



void Interface::getFluidImpactOnDynamicBodies(std::vector<Vector3d>& forces, std::vector<Vector3d>& moments,
                                              const std::vector<Vector3d>& reduction_factors){
    //read from engine
    std::vector<SPH::Vector3d> sph_forces,sph_moments;

    std::vector<SPH::Vector3d> reduction_factor_SPH;
    for (int i=0;i<reduction_factors.size();++i){
        reduction_factor_SPH.push_back(vector3dToSPH3d(reduction_factors[i]));
    }
    SingletonDFSPHCUDA::getFluid()->getFluidImpactOnDynamicBodies(sph_forces,sph_moments,reduction_factor_SPH);

    //init the buffers for the avg on first run
    static std::vector<std::vector<Vector3d> > old_forces;
    static std::vector<std::vector<Vector3d> > old_moments;
    static bool first_time=true;
    if (first_time){
        first_time=false;

        old_forces.resize(sph_forces.size());
        old_moments.resize(sph_forces.size());
    }

    //compute the avg and set the result
    int timestep_for_avg=10;
    forces.clear();
    moments.clear();
    for (int i=0;i<sph_forces.size();++i){
        old_forces[i].push_back(vectorSPH3dTo3d(sph_forces[i]));
        old_moments[i].push_back(vectorSPH3dTo3d(sph_moments[i]));

        if (old_forces[i].size()>timestep_for_avg){
            old_forces[i].erase(old_forces[i].begin(),old_forces[i].begin()+1);
            old_moments[i].erase(old_moments[i].begin(),old_moments[i].begin()+1);
        }

        Vector3d avg_force=Vector3d(0,0,0);
        Vector3d avg_moments=Vector3d(0,0,0);
        for (int j=0;j<old_forces[i].size();++j){
            avg_force+=old_forces[i][j];
            avg_moments+=old_moments[i][j];
        }

        avg_force/=old_forces[i].size();
        avg_moments/=old_moments[i].size();

        forces.push_back(avg_force);
        moments.push_back(avg_moments);

        /*
        std::cout<<"force on body "<<i<<":   "<<avg_force.x<<"  "<<avg_force.y<<"  "<<avg_force.z<<"   ("<<avg_force.length()<<
                   ")"<<"      with a sampling ofN forces: "<<old_forces[i].size()<<std::endl;
        //*/
        if (false&&i==0){
            std::string filename = "force_forces.csv";
            static bool first_time=true;
            if (first_time) {
                first_time=false;
                std::remove(filename.c_str());
            }
            std::ofstream myfile;
            myfile.open(filename, std::ios_base::app);
            if (myfile.is_open()) {
                myfile <<avg_force.x<<"  "<<avg_force.y<<"  "<<avg_force.z<< std::endl;;
                myfile.close();
            }
            else {
                std::cout << "failed to open file: " << filename << "   reason: " << std::strerror(errno) << std::endl;
            }
        }
    }



}


void Interface::getFluidBoyancyOnDynamicBodies(std::vector<Vector3d>& forces, std::vector<Point3d>& pts_appli){

    std::vector<SPH::Vector3d> sph_forces, sph_pts_appli;

    SingletonDFSPHCUDA::getFluid()->getFluidBoyancyOnDynamicBodies(sph_forces,sph_pts_appli);

    forces.clear();
    pts_appli.clear();
    for (int i=0;i<sph_forces.size();++i){
        forces.push_back(vectorSPH3dTo3d(sph_forces[i]));
        pts_appli.push_back(vectorSPH3dTo3d(sph_pts_appli[i]));
    }

}

void Interface::fluidSimulationStep(){
    //std::cout<<"step fluid"<<std::endl;
    SingletonDFSPHCUDA::getFluid()->step();
}

void Interface::updateTimeStepDuration(double duration){
    SingletonDFSPHCUDA::getFluid()->updateTimeStepDuration(duration);
}

void Interface::handleDynamicBodiesPause(bool pause){
    SingletonDFSPHCUDA::getFluid()->handleDynamicBodiesPause(pause);
}


void Interface::zeroFluidVelocities(){
    SingletonDFSPHCUDA::getFluid()->zeroFluidVelocities();
}

bool Interface::moveFluidSimulation(Point3d target_Position){
    //get the center
    SPH::Vector3d sim_center=SingletonDFSPHCUDA::getFluid()->getSimulationCenter();

    //std::cout<<"Interface::moveFluidSimulation sim center: "<<
    //           sim_center.x<<"  "<<sim_center.z<<std::endl;


    SPH::Vector3d delta=vector3dToSPH3d(target_Position)-sim_center;

    //compute the direction
    SPH::Vector3d movement=SPH::Vector3d(0,0,0);
    if(std::abs(delta.x)>0.1){
        movement.x=delta.x/std::abs(delta.x);
    }else if(std::abs(delta.z)>0.1){
        movement.z=delta.z/std::abs(delta.z);
    }

    /*
    if(movement.squaredNorm()>0)
    {
        std::cout<<"Interface::moveFluidSimulation required movement x,z (actual delta x,z): "<<
                   movement.x<<"  "<<movement.z<<"  ("<<delta.x<<"  "<<delta.z<<")"<<std::endl;
    }
    ///*/

    //move the simulation
    if (movement.squaredNorm()>0.5){
        SingletonDFSPHCUDA::getFluid()->handleSimulationMovement(movement);
        return true;
    }

    return false;
}



void shaderBegin(const float* col, float particleRadius, float renderMaxVelocity, int type=0){
    SPH::Shader& m_shader=SingletonDFSPHCUDA::getShader(type);
    m_shader.begin();

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glUniform1f(m_shader.getUniform("viewport_width"), (float)viewport[2]);
    glUniform1f(m_shader.getUniform("radius"), (float)particleRadius);
    glUniform1f(m_shader.getUniform("max_velocity"), (GLfloat) renderMaxVelocity);
    glUniform3fv(m_shader.getUniform("color"), 1, col);

    GLfloat matrix[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
    glUniformMatrix4fv(m_shader.getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
    GLfloat pmatrix[16];
    glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
    glUniformMatrix4fv(m_shader.getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);

    glEnable(GL_DEPTH_TEST);
    // Point sprites do not have to be explicitly enabled since OpenGL 3.2 where
    // they are enabled by default. Moreover GL_POINT_SPRITE is deprecate and only
    // supported before OpenGL 3.2 or with compatibility profile enabled.
    glEnable(GL_POINT_SPRITE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointParameterf(GL_POINT_SPRITE_COORD_ORIGIN, GL_LOWER_LEFT);

}

void shaderEnd(){
    SingletonDFSPHCUDA::getShader().end();
}


void Interface::drawParticles(bool drawFluid, bool drawBodies, bool drawBoundaries, bool transparent_fluid){

    float fluidColor[4] = { 0.3f, 0.5f, 0.9f, 1.0f };
    shaderBegin(&fluidColor[0],0.025,25);

    float wallColor[4] = { 0.1f, 0.6f, 0.6f, 1.0f };
    glUniform3fv(SingletonDFSPHCUDA::getShader().getUniform("color"), 1, &wallColor[0]);

    if (drawBodies||drawBoundaries){
        SingletonDFSPHCUDA::getFluid()->renderBoundaries(drawBoundaries);
    }

    shaderEnd();

    int shader_type=0;
    if (transparent_fluid){
        glDepthMask(GL_FALSE);
        shader_type=1;
    }

    shaderBegin(&fluidColor[0],0.025,25,shader_type);


    if (drawFluid){
        SingletonDFSPHCUDA::getFluid()->renderFluid();
    }


    shaderEnd();

    if (transparent_fluid){
        glDepthMask(GL_TRUE);
    }

}

void Interface::handleFLuidLevelControl(float level){
    SingletonDFSPHCUDA::getFluid()->handleFLuidLevelControl(level);
}

float Interface::getFluidLevel(){
    return SingletonDFSPHCUDA::getFluid()->getFluidLevel();
}


