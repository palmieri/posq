/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Social Robotics Laboratory, University of Freiburg.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Luigi Palmieri */

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

/** \brief Definition of the namespaces */

namespace ob = ompl::base;
namespace oc = ompl::control;

/** \brief length of the wheels axis */
const double B = 0.54;

/** \brief Integration Time Step  */
const double DT = 0.1;

/** \brief Max Size of the Control (HacK: in this case equal to the dimension) */
const int MAX_SIZE = 10001;

/** \brief Value used to mark the end of the vector, no used at the moment */
const double END_CONTROLS = 2000;

/** \brief Internal definition of the Unicycle state type */
struct UnicycleState {
    // pose
    double x_;
    double y_;
    double yaw_;
    UnicycleState(void)
    {
        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;
    }
    UnicycleState(double xx, double yy, double zz) : x_(xx), y_(yy),yaw_(zz)
    {
    }
    UnicycleState(double* p)
    {
        x_ = p[0];
        y_ = p[1];
        yaw_ = p[2];
    }
    UnicycleState& operator=(UnicycleState const& copy)
    {
        x_ = copy.x_;
        y_ = copy.y_;
        yaw_ = copy.yaw_;
        return *this;
    }
    UnicycleState(const UnicycleState& copy)
    {
        x_ = copy.x_;
        y_ = copy.y_;
        yaw_ = copy.yaw_;
    }
    ~UnicycleState(void)
    {
    } 
};

/** \brief Internal definition of the Unicycle control type*/
struct UnicycleControl
{
    // v, translational velocity
    double v_;
    // w, rotational velocity
    double w_;

    UnicycleControl(void)
    {
        v_ = 0.0;
        w_ = 0.0;
    }
    UnicycleControl(double vv, double ww) : v_(vv), w_(ww)
    {
    }

    UnicycleControl(double* p)
    {
        v_ = p[0];
        w_ = p[1];
    }

    UnicycleControl& operator=(UnicycleControl const& copy)
    {
        v_ = copy.v_;
        w_ = copy.w_;
        return *this;
    }

    UnicycleControl(const UnicycleControl& copy)
    {
        v_ = copy.v_;
        w_ = copy.w_;
    }

    ~UnicycleControl(void)
    {
    } 
};

/** \brief POSQ class that implements the POSQ steering function */
class POSQ : public oc::StatePropagator
{
public:
    /** \brief Internal Results of the integration */
    double *result_;
    double *intRes_;

    /** \brief Internal Counter to access the current control to propagate */
    int *ind_;

    /** \brief Current control to propagate */
    oc::Control *controlRes_;

    /** \brief ControlSpace and SpaceInformation */
    oc::RealVectorControlSpace *controlSpace_;
    oc::SpaceInformationPtr si_;

    POSQ(const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
    {
        this->si_ = si;
        ob::StateSpacePtr space(new ob::SE2StateSpace());
        controlSpace_ = new oc::RealVectorControlSpace(space,MAX_SIZE);
        controlRes_ = (*controlSpace_).allocControl();

        result_ = (double*)malloc(sizeof(double)*5);
        intRes_ = (double*)malloc(sizeof(double)*5);
        ind_ = (int*)malloc(sizeof(int)*1);
    }

    /** \brief Set the current result of the integration */
    void setRes(double *r) const
    {
        this->intRes_[0] = r[0];
        this->intRes_[1] = r[1];
        this->intRes_[2] = r[2];
        this->intRes_[3] = r[3];
        this->intRes_[4] = r[4];
    }

    /** \brief Reset internal counter to i */
    void resetIndControl(int i)  const
    {
       ind_[0] = i;
    }

    /** \brief Increment internal counter */
    void incrementIndControl(void) const
    {
       ind_[0] = ind_[0] + 1;
    }

    /** \brief Propagate the model of the system forward, starting at a given state, with a given control, for a given number of steps.
        \param state the state to start at
        \param control the control to apply, in this Class the Control is not read from the propagate function but it is saved internally as control_res
        \param In this case Duration of the single propagation
        \param result the state at the end of the propagation */

    void propagate(const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const
    { 
        const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();
        const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
        oc::RealVectorControlSpace::ControlType *rcontrol = static_cast< oc::RealVectorControlSpace::ControlType*>(controlRes_);
        // controlSpace->printControl(rcontrol, std::cout);

        double x = pos[0];
        double y = pos[1];
        double yaw = rot;

        yaw = setAngleToRange(yaw,0);
        
        double* step_v = controlSpace_->getValueAddressAtIndex(rcontrol,2*ind_[0]);
        double* step_w = controlSpace_->getValueAddressAtIndex(rcontrol,2*ind_[0]+1);

        

        if ((*step_v) == END_CONTROLS || (*step_w) == END_CONTROLS)
        {
            
            result->as<ob::SE2StateSpace::StateType>()->setXY(x,y);
            result->as<ob::SE2StateSpace::StateType>()->setYaw(yaw);
            return ;
        }

        x =  x + (*step_v) * duration * cos(yaw),
        y =  y + (*step_v) * duration * sin(yaw);
        yaw = yaw + (*step_w) * duration;
        yaw = setAngleToRange(yaw,0);

        incrementIndControl();
        
        result->as<ob::SE2StateSpace::StateType>()->setXY(x,y);
        result->as<ob::SE2StateSpace::StateType>()->setYaw(yaw);
    }

    /** \brief Normalize the angle a rispect to the minimum angle mina.
        \param double a 
        \param double mina 
    */
    double normAngle(double a, double mina) const
    {
        double ap,minap;
        ap = a;
        minap = mina;
        
        while (ap >= (minap + M_PI * 2))
        {
            ap = ap - M_PI * 2;
        }

        while (ap < minap)
        {
            ap = ap + M_PI * 2;
        }
        
        return ap;
    }

    /** \brief Set the angle a in the range [min, min+2*M_PI].
        \param double alpha 
        \param double min
    */
    double setAngleToRange(double alpha, double min) const
    {
        while (alpha >= min + 2.0 * M_PI)
        {
            alpha -= 2.0 * M_PI;
        }

        while (alpha < min)
        {
            alpha += 2.0 * M_PI;
        }
        
        return alpha;
    }

    /** \brief Single step of the Steer Function.
        \param double x_c, initial x-coord
        \param double y_c, initial y-coord
        \param double t_c, initial orientation
        \param double x_end, final x-coord.
        \param double y_end, final y-coord.
        \param double t_end, final orientation
        \param double c_t, integration time step
        \param double b, length of the wheel axis
        \param double dir, direction of the robot dir==1 forward
    */
    double* posControlStep (double x_c, double y_c, double t_c, 
                   double x_end, double y_end, double t_end, double ct, double b, int dir) const
    {    
        /** This function will generate a vector of double as output:
        *  [0] Vl velocity of the left wheel;
        *  [1] Vr velocity of the right wheel;
        *  [2] V translational Velocity;
        *  [3] W Angular Velocity.
        *  [4] EOT End Of Trajectory
        **/
        static double oldBeta;
        
        double Krho,Kalpha,Kbeta,Kv,Vmax,RhoEndCondition;
        
        Kalpha  = 6.91;
        Kbeta   = -1;
        

        /** Read the paper and see how to set the proper gain values
           
           double Kphi;
           Kphi    = -1;
        **/


        Krho = 0.2;
        Kv   = 5.9;

        Vmax = Krho;
        RhoEndCondition = 0.15;

        if (ct == 0)
            oldBeta = 0;
        
        double dx,dy,rho,fRho,alpha,phi,beta,v,w,vl,vr,eot;
        
        // rho
        eot = 1;
        dx = x_end - x_c;
        dy = y_end - y_c;
        rho = sqrt(dx*dx + dy*dy);
        fRho = rho;

        if (fRho > (Vmax/Krho))
            fRho = Vmax/Krho;
        
        // alpha
        alpha = atan2(dy,dx) - t_c;
        alpha = normAngle(alpha,-M_PI);

        // direction
        if (dir == 0)
        {
            if (alpha > (M_PI/2))
            {
                fRho = -fRho;
                alpha = alpha - M_PI;
            }
            else if (alpha <= -M_PI/2)
            {
                fRho = -fRho;
                alpha = alpha + M_PI;
            }
        }
        else if (dir == -1)
        {
            fRho = -fRho;
            alpha = alpha + M_PI;
            
            if (alpha > M_PI)
            {
                alpha = alpha - 2 * M_PI;
            }
        }

        // phi
        phi = t_end - t_c;
        phi = normAngle(phi, -M_PI);
        beta = normAngle(phi-alpha, -M_PI);

        if ((abs(oldBeta - beta) > M_PI))
            beta = oldBeta;

        oldBeta = beta;

        //set speed
        v = Krho * tanh(Kv * fRho);
        w = Kalpha * alpha + Kbeta*beta;

        if (rho < RhoEndCondition)
        {
            eot = 1;
        }
        else
        {
            eot = 0;
        }

        if (eot)
            w = 0.;

        //Convert speed to wheel speed
        vl = v - w * b/2;
        if (abs(vl) > Vmax)
        {
            if(vl < 0)
                vl = Vmax * -1;
            else
                vl = Vmax;
        }
        
        vr = v + w * b/2;
        if(abs(vr) > Vmax)
        {
            if(vr < 0)
                vr = Vmax * -1;
            else
                vr = Vmax;
        }
        
        result_[0] = vl;
        result_[1] = vr;
        result_[2] = (vl+vr)/2;
        result_[3] = (vr-vl)/b;
        result_[4] = eot;

        return result_;
    }

    /** \brief Propagate the model of the system forward, starting at a given state, with a given control, for a given number of steps.
        \param state from Initial state of the steering
        \param state to Ending state of the steering
        \param control c_result resulting vector of controls
        \param double duration, number of control steps needed to steer the robot
    */
       
    bool steer(const ob::State* from, const ob::State* to, oc::Control* c_result, double& duration) const
    {
        double sl,sr,oldSl,oldSr,t,eot,dSl,dSr,dSm,dSd,vl,vr,enc_l,enc_r,dir;
        dir = 1;
        enc_l = 0;
        enc_r = 0;
        sl = 0;
        sr = 0;
        oldSl = 0;
        oldSr = 0;
        eot = 0;
        t = 0;
        vl = 0;
        vr = 0;

        double x,y,th;
        double x_fin,y_fin,th_fin;

        x = from->as<ob::SE2StateSpace::StateType>()->getX();
        y = from->as<ob::SE2StateSpace::StateType>()->getY();
        th = from->as<ob::SE2StateSpace::StateType>()->getYaw();

        x_fin = to->as<ob::SE2StateSpace::StateType>()->getX();
        y_fin = to->as<ob::SE2StateSpace::StateType>()->getY();
        th_fin = to->as<ob::SE2StateSpace::StateType>()->getYaw();
        
        double vv,ww;
        vv = 0;
        ww = 0;

        std::vector<UnicycleControl> controls;
        std::vector<UnicycleState> states;
        controls.clear();
        states.clear();

        int n_steps;
        n_steps = 0;

        while (eot == 0){
            // calculate distance for both wheels
            dSl = sl - oldSl;
            dSr = sr - oldSr;
            dSm = (dSl + dSr)/2;
            dSd = (dSr - dSl)/B;
            x = x + dSm * cos(th + dSd/2);
            y = y + dSm * sin(th + dSd/2);
            th = normAngle(th + dSd, -M_PI);
            // intRes= posControlStep (x,y,th,x_fin,y_fin,th_fin, t,b,dir);
            setRes(posControlStep(x,y,th,x_fin,y_fin,th_fin,t,B,dir));
            //Save the velocity commands,eot
            vv = intRes_[2];
            ww = intRes_[3];
            eot = intRes_[4];
            vl = intRes_[0];
            vr = intRes_[1];
            //Increase the timer
            t = t + DT;
            // Count the number of steps
            n_steps++;
            // keep track of previous wheel position
            oldSl = sl;
            oldSr = sr;
            // increase encoder values
            enc_l = enc_l + DT * vl;
            enc_r = enc_r + DT * vr;
            sl = enc_l;
            sr = enc_r;
            if (eot == 1)
            {
                // Add current values to the Trajectory
                states.push_back(UnicycleState(x,y,th));
                controls.push_back(UnicycleControl(vv,ww));

                // save the last state!!!
                double xf,yf,yawf,vf,wf;
                vf = intRes_[2];
                wf = intRes_[3];
                dSl = sl - oldSl;
                dSr = sr - oldSr;
                dSm = (dSl + dSr)/2;
                dSd = (dSr - dSl)/B;
                xf = x + dSm * cos(th + dSd/2);
                yf = y + dSm * sin(th + dSd/2);
                yawf = normAngle(th + dSd, -M_PI);

                states.push_back(UnicycleState(xf,yf,yawf));
                controls.push_back(UnicycleControl(vf,wf));

            }
            else
            {
                // Add current values to the Trajectory
                UnicycleState s;
                s.x_ = x;
                s.y_ = y;
                s.yaw_ = th;
                states.push_back(s);

                UnicycleControl c;
                c.v_ = vv;
                c.w_ = ww;
                controls.push_back(c);
            }
        }

        (*controlSpace_).nullControl(controlRes_);
        oc::RealVectorControlSpace::ControlType *rcontrol = static_cast< oc::RealVectorControlSpace::ControlType*>(controlRes_);


        for (int  i = 0 ; i < (int)controls.size() ; i++)
        {

            (*rcontrol).values[2*i]   =  controls[i].v_;
            (*rcontrol).values[2*i+1] =  controls[i].w_;
        }

        (*rcontrol).values[2*(int)controls.size()] = END_CONTROLS;

        c_result = (*controlSpace_).allocControl();
        controlSpace_->copyControl(c_result, controlRes_);
        // controlSpace->printControl(c_result,std::cout);
        duration = (int)(controls.size()/2);
        resetIndControl(0);
        
        return true;
    }

    /** \brief Returns true */
    bool canSteer(void) const
    {
        return true; 
    }

};
