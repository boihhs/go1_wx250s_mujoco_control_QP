

#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <iostream>
#include "utils.h"
#include "go1_fk.h"
#include "MPC_QP.h"
#include "Cartesian_PD_End.h"
#include "contact_jacobian.h"
#include "ContactJacobian_Arm.h"
#include "getVeloctyArm_feet.h"
#include "Traj_Command_01M.h"
#include "CartesianControl_01.h"
#include "globals.h"
#include "IK_PD_End.h"
#include <fstream>
#include <random>
#include "Traj_Command_Goal.h"
#include <chrono>


using namespace Eigen;

char filename[] = "/home/leo-benaharon/Documents/mujoco-3.2.6-linux-x86_64/mujoco-3.2.6/my_project/template/go1_wx250s/scene.xml";

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update buttoCartesian_PD_Endn state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// main function
int main(int argc, const char** argv)
{
    // Open a file to record the time and trunk position
    std::ofstream posFile("robot_position.csv");
    if (!posFile.is_open()) {
        std::cerr << "Error: could not open robot_position.csv for writing." << std::endl;
        return 1;
    }
    posFile << "time,xArm,yArm,zArm,xtraj,ytraj,ztraj,xGoal,yGoal,zGoal,xPArm,yPArm,zPArm\n";

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_x(-1.0, 1.0);
    std::uniform_real_distribution<double> dist_y(-1.0, 1.0);
    std::uniform_real_distribution<double> dist_z(0.3, 1.0);  // e.g., above ground

    int site_id = mj_name2id(m, mjOBJ_SITE, "red_sphere_site");
    if (site_id < 0) {
        std::cerr << "Error: red_sphere_site not found!" << std::endl;
 
    }

    

    double random_x = dist_x(gen);
    double random_y = dist_y(gen);
    double random_z = dist_z(gen);

    // Update the body position in d->qpos.
    // (Assuming the body has 7 DoF (position + quaternion) for a free body.)
    d->site_xpos[3 * site_id + 0] = random_x;
    d->site_xpos[3 * site_id + 1] = random_y;
    d->site_xpos[3 * site_id + 2] = random_z;

    Vector3d goal;
    goal << random_x, random_y, random_z;
    Vector3d lastPos;
    lastPos << 0.4507, 0.0, 0.7187;

    // You may leave the quaternion unchanged if you do not need to update orientation.

    // Call mj_forward to update all computed quantities.
    mj_forward(m, d);
    //mjv_updateScene(m, d, &opt, NULL, &pert, &scn);



    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 0.000000};
    // cam.azimuth = arr_view[0];
    // cam.elevation = arr_view[1];
    // cam.distance = arr_view[2];
    // cam.lookat[0] = arr_view[3];
    // cam.lookat[1] = arr_view[4];
    // cam.lookat[2] = arr_view[5];
    //double t = 11.6970;
    VectorXd qEveryPos = VectorXd::Zero(18);
    VectorXd qEveryVel = VectorXd::Zero(18);
    VectorXd q = VectorXd::Zero(12);
    VectorXd dq = VectorXd::Zero(12);
    VectorXd qArm = VectorXd::Zero(6);
    VectorXd dqArm = VectorXd::Zero(6);
    Vector4d quatVec = Vector4d::Zero(4);
    VectorXd xin = VectorXd::Zero(19); // [eular, pos, ang vel, vel, 1, arm pos, arm vel]
    VectorXd xin2 = VectorXd::Zero(19);
    VectorXd x = VectorXd::Zero(19); // [quat, pos, ang vel, vel, arm pos, arm vel]
    VectorXd tau = VectorXd::Zero(18);
    VectorXd F(15);

    Vector3d F_Tatile = Vector3d::Zero(3);

    // Get the transformed COM/Inertia values
    computeTransformations();

    double lastTime = 0;



    while( !glfwWindowShouldClose(window))
   //bool hello = true;
    //while (hello)
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        //hello = false;

        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }
        
       
        for (int i = 1; i < m->njnt && i < 19; i++) {
            const char* joint_name = mj_id2name(m, mjOBJ_JOINT, i); // Get joint name
            //printf("Joint name: %s\n", joint_name);
            int joint_qpos_addr = m->jnt_qposadr[i]; // Address in qpos array

            
            qEveryPos(i-1) = d->qpos[joint_qpos_addr];
            qEveryVel(i-1) = d->qvel[joint_qpos_addr];

        }
        
        q = qEveryPos.segment(0, 12);
        qArm = qEveryPos.segment(12, 6);

        dq = qEveryVel.segment(0, 12);
        dqArm = qEveryVel.segment(12, 6);

        int body_id = mj_name2id(m, mjOBJ_BODY, "trunk");
        mjtNum* pos = d->xpos + 3 * body_id; // [pos]
        mjtNum* vel = d->cvel + 6 * body_id; // [angular vel, vel]
        mjtNum* quat = d->xquat + 4 * body_id; //[quat]


        int arm_id = mj_name2id(m, mjOBJ_BODY, "wx250s/gripper_link");
        mjtNum* posArm = d->xpos + 3 * arm_id; // [pos]
        

        double t = d->time;


        ///////////////////
        // Make input values
        /////////////////////

        x << quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], 0, 0, 0, 0, 0, 0;
        
        //std::cout << "\n x:  " << x.transpose() << std::endl;
        //std::cout << "\n q: " << q.transpose() << std::endl;

        //std::cout << "\n qArm: " << qArm.transpose() << std::endl;
    
        
        Vector4d quatn = x.segment(0, 4);
        Vector3d eular = rotm2eul(quat2rotm(quatn/quatn.norm()));
        
        xin.segment(0, 3) = eular.reverse();
        xin.segment(3, 9) = x.segment(4, 9);
        xin(12) = 1;
      
        Vector3d F_Tatile(0, 0, 0);
        Vector3d pArm(posArm[0],posArm[1],posArm[2]);
        
        //double t = 11;

        //std::cout << "\n t: " << t << std::endl; 
        
        //std::cout << "\n q: "  << q.transpose() << std::endl;
        //std::cout << "\n qArm: "  << qArm.transpose() << std::endl;

        VectorXd PosVel = FK(xin, q, qArm);
        
        //VectorXd VelLegs = legVelocity(xin, q, dq);
        
        Vector3d VelArm = armVelocity(xin, qArm, dqArm);
        

        //PosVel.segment(3,3) = VelLegs.segment(0,3);
        //PosVel.segment(9,3) = VelLegs.segment(3,3);
        //PosVel.segment(15,3) = VelLegs.segment(6,3);
        //PosVel.segment(21,3) = VelLegs.segment(9,3);
        PosVel.segment(27,3) = VelArm;
    
        //std::cout << "\n eular: "  << eular.transpose() << std::endl;
        // Log time and position
        
        xin.segment(13, 3) = pArm;
        x.segment(13, 3) = PosVel.segment(24, 3);

        if (t - lastTime > 10){
            lastPos = goal;
            random_x = dist_x(gen);
            random_y = dist_y(gen);
            random_z = dist_z(gen);
            lastTime = t;
            goal << random_x, random_y, random_z;
            
        }

        ////////////////////////
        // Run Commands
        /////////////////////////
        
        computeInertialParams(qArm); // Compute full body inertia and COM
        
        //VectorXd xdes = Traj_Command_01M(t, xin);
        VectorXd xdes = Traj_Command_Goal2(t, xin, goal, lastPos);
        std::cout << "\n t: " << t << std::endl;
        std::cout << "\n Arm traj: \n" << xdes.segment(13, 3).transpose() << std::endl;
        
        VectorXd tauAirLeg = CartesianControl_01(t, xin, q, xdes, PosVel.segment(0,24), qArm);   
   
        //std::cout << "\n tauArm: "  << tauArm.transpose() << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        VectorXd F = MPC_QP(xin, PosVel, qArm, F_Tatile, t, goal, lastPos);
        auto end = std::chrono::high_resolution_clock::now(); // End time
        std::chrono::duration<double> duration = end - start; // Compute duration
        posFile << t << "," << duration.count() << "," << posArm[1] << "," << posArm[2]<<   "," << xdes(13) << "," << xdes(14) << "," << xdes(15) <<   "," << goal(0) << "," << goal(1) << "," << goal(2) << "," << PosVel(24) << "," << PosVel(25) << "," << PosVel(26) << "\n";
        std::cout << "Function execution time: " << duration.count() << " seconds \n" << std::endl;

        //std::cout << "\n F: " << F.transpose();
        
        VectorXd FArm = F.segment(12,3);
       
        VectorXd F_c = F.segment(0,12);
        VectorXd tauMain = ContactJacobian_01(xin, q, F_c);
        VectorXd tau(18);
        tau.setZero();
        tau.segment(0,12) = tauMain.segment(6,12) + tauAirLeg;
        

        //std::cout << "\n x: \n" << x.transpose() << std::endl;
        //std::cout << "\n qArm: \n" << qArm.transpose() << std::endl;
        //std::cout << "\n dqArm: \n" << dqArm.transpose() << std::endl;
        //std::cout << "\n FArm: \n" << FArm.transpose() << std::endl;
        //std::cout << "\n t: \n" << t << std::endl;
        //std::cout << "\n ContactJacobian_Arm: \n" << ContactJacobian_Arm(xin,qArm, dqArm,FArm).transpose() << std::endl;
        //std::cout << "\n IK_PD_End: \n" << IK_PD_End(xin, t, qArm).transpose() << std::endl;
        VectorXd tauArm = ContactJacobian_Arm(xin,qArm, dqArm,FArm) + IK_PD_End(xin, t, qArm, dqArm, goal, lastPos);
    
        

        tau.segment(12, 6) = tauArm;
    
        
        for (int i = 0; i < m->njnt && i < 18; i++) {
            if ((d->time) > 0){
                d->ctrl[i] = tau(i);
            }else{
                d->ctrl[0] = 0;
            }
            
            //const char* joint_name = mj_id2name(m, mjOBJ_JOINT, i); // Get joint name
            //printf("Joint name: %s\n", joint_name);
            //d->ctrl[i] = 0;
            
            
        
        }

        d->site_xpos[3 * site_id + 0] = -50;
        d->site_xpos[3 * site_id + 1] = -50;
        d->site_xpos[3 * site_id + 2] = -50;

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // Close file after simulation ends
    posFile.close();

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
