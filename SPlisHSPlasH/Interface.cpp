#include "Interface.h"
#include <iostream>
#include "Shader.h"
#include "Utils/Utils.h"
#include "SPlisHSPlasH/DFSPH/DFSPH_CUDA.h"

//first I need a special function that will contain the fluid structure and that will be used to launch all the calls
class SingletonDFSPHCUDA{
public:
    static SPH::DFSPHCUDA* getFluid(){
        static SPH::DFSPHCUDA* fluid=NULL;
        if (fluid==NULL){
            fluid=new SPH::DFSPHCUDA();
        }
        return fluid;
    }

    static SPH::Shader& getShader(){
        static SPH::Shader m_shader;
        static bool first_pass=true;
        if (first_pass){
            first_pass=false;

            std::string config_folder_path=get_folder_path("configuration_data",4);

            std::string vertFile = config_folder_path+ "shaders/vs_points.glsl";
            std::string fragFile = config_folder_path+ "shaders/fs_points.glsl";

            m_shader.compileShaderFile(GL_VERTEX_SHADER, vertFile);
            m_shader.compileShaderFile(GL_FRAGMENT_SHADER, fragFile);
            m_shader.createAndLinkProgram();
            m_shader.begin();
            m_shader.addUniform("modelview_matrix");
            m_shader.addUniform("projection_matrix");
            m_shader.addUniform("radius");
            m_shader.addUniform("viewport_width");
            m_shader.addUniform("color");
            m_shader.addUniform("projection_radius");
            m_shader.addUniform("max_velocity");
            m_shader.end();

        }
        return m_shader;
    }
};

void Interface::initFluid(double timeStep){
    //for now I'll init it by loading from file
    SingletonDFSPHCUDA::getFluid()->handleSimulationLoad(true,false,false,false,true,false);

    //also init the shader
     SingletonDFSPHCUDA::getShader();


     handleDynamicBodiesPause(true);

    //Beause it is impossible o just start the simulation with a large time step immediatly (idk why btw)
    //the solution I'll use will be to do a stabilisaion period of 10 timestepswhen initalising the fluid
    //during those steps I'll start with a 1ms duration and go to the desired duration
    /*
     handleDynamicBodiesPause(true);
     int nb_iter_init=10;
    double delta=(timeStep-0.001)/nb_iter_init;
    for (int i=0;i<10;++i){
        updateTimeStepDuration(0.001+delta*i);
        fluidSimulationStep();
    }
    updateTimeStepDuration(timeStep);
    fluidSimulationStep();
    //*/

     handleDynamicBodiesPause(false);
}

void Interface::updateDynamicBodies(const std::vector<SPH::DynamicBody> &vect_new_info){
    SingletonDFSPHCUDA::getFluid()->updateRigidBodies(vect_new_info);
}

void Interface::forceUpdateDynamicBodies(){
    SingletonDFSPHCUDA::getFluid()->forceUpdateRigidBodies();
}



void Interface::getFluidImpactOnDynamicBodies(std::vector<Vector3d>& forces, std::vector<Vector3d>& moments){
    std::vector<SPH::Vector3d> sph_forces,sph_moments;

    SingletonDFSPHCUDA::getFluid()->getFluidImpactOnDynamicBodies(sph_forces,sph_moments);

    forces.clear();
    moments.clear();
    for (int i=0;i<sph_forces.size();++i){
        forces.push_back(vectorSPH3dTo3d(sph_forces[i]));
        moments.push_back(vectorSPH3dTo3d(sph_moments[i]));
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

void Interface::moveFluidSimulation(Point3d target_Position){

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

    if(movement.squaredNorm()>0)
    {
        std::cout<<"Interface::moveFluidSimulation required movement x,z (actual delta x,z): "<<
                   movement.x<<"  "<<movement.z<<"  ("<<delta.x<<"  "<<delta.z<<")"<<std::endl;
    }

    //move the simulation
    if (movement.squaredNorm()>0.5){
        SingletonDFSPHCUDA::getFluid()->handleSimulationMovement(movement);
    }
}



void shaderBegin(const float* col, float particleRadius, float renderMaxVelocity){
    SPH::Shader& m_shader=SingletonDFSPHCUDA::getShader();
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


void Interface::drawParticles(bool drawFluid, bool drawBodies, bool drawBoundaries){

    float fluidColor[4] = { 0.3f, 0.5f, 0.9f, 1.0f };
    shaderBegin(&fluidColor[0],0.025,25);


    if (drawFluid){
        SingletonDFSPHCUDA::getFluid()->renderFluid();
    }

    float wallColor[4] = { 0.1f, 0.6f, 0.6f, 1.0f };
    glUniform3fv(SingletonDFSPHCUDA::getShader().getUniform("color"), 1, &wallColor[0]);

    if (drawBodies||drawBoundaries){
        SingletonDFSPHCUDA::getFluid()->renderBoundaries(drawBoundaries);
    }

    shaderEnd();

}


