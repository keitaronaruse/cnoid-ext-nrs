#include <cnoid/SimpleController>
#include <fstream>
#include <iomanip>
#include <iostream>

class NrsMassTSaveController : public cnoid::SimpleController
{
    cnoid::Link *t_link, *b_link;
    cnoid::SimpleControllerIO* io;
    std::ofstream ofs;
    double dt;
    double t;

public:
    virtual bool initialize(cnoid::SimpleControllerIO* r_io) override
    {
        io = r_io;
        b_link = io->body()->rootLink();
        io->enableInput(b_link,
                        cnoid::Link::LinkPosition | cnoid::Link::LinkTwist);
        t_link = io->body()->link("Mass");
        io->enableInput(t_link,
                        cnoid::Link::LinkPosition | cnoid::Link::LinkTwist);
        dt = io->timeStep();

        return true;
    }

    virtual bool start() override
    {
        ofs = std::ofstream(io->optionString());
        t = 0.0;
        return true;
    }

    virtual bool control() override
    {
        //  Base link
        cnoid::Isometry3 b_T = b_link->position();
        cnoid::Vector3 b_t = b_T.translation();
        cnoid::Matrix3 b_R = b_T.rotation();
        //  Target link
        cnoid::Isometry3 t_T = t_link->position();
        cnoid::Vector3 t_t = t_T.translation();
        cnoid::Matrix3 t_R = t_T.rotation();
        cnoid::Vector3 t_dv = t_link->v();
        cnoid::Vector3 t_dw = t_link->w();

        ofs << std::fixed << std::setprecision(6) << t << " " << b_t.x() << " "
            << b_t.y() << " " << b_t.z() << " " << t_t.x() << " " << t_t.y()
            << " " << t_t.z() << " " << t_dv.x() << " " << t_dv.y() << " "
            << t_dv.z() << " " << std::endl;

        t += dt;
        return true;
    }

    virtual void stop() override { ofs.close(); }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(NrsMassTSaveController)
