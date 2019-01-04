#include <ode/ode.h>
#include <Klampt/Simulation/ODERigidObject.h>
#include <Klampt/Simulation/ODECommon.h>
#include <Klampt/Simulation/ODECustomGeometry.h>
#include <gtest/gtest.h>

class testODERigidObject: public ::testing::Test
{
public:

protected:
    RigidObject r_obj;
    ODERigidObject ode_r_obj;

    testODERigidObject() : ode_r_obj(r_obj)
    {
        r_obj.Load("tests/objects/block.obj");
        Math3D::Vector3 com_position(0.023, 0.045, 0.061);
        r_obj.com.set(com_position);

        dInitODE();
        InitODECustomGeometry();

        dWorldID worldID = dWorldCreate();
        //contactGroupID = dJointGroupCreate(0);
        dSpaceID envSpaceID = dSimpleSpaceCreate(0);

        dWorldSetERP(worldID,0.95);
        dWorldSetCFM(worldID,1e-6);
        dWorldSetGravity(worldID,0,0,-9.81);

        ode_r_obj.Create(worldID, envSpaceID, false);
    }

    virtual ~testODERigidObject() {
        ode_r_obj.Clear();
        dCloseODE();
    }

    virtual void SetUp() {
        Math3D::Matrix3 w_R_obj;
        w_R_obj.setIdentity();
        w_R_obj.setRotateZ(0.13);
        w_R_obj.setRotateY(0.23);
        w_R_obj.setRotateZ(0.35);

        Math3D::Vector3 w_p_obj(1.381, 0.931, 2.148);

        Math3D::RigidTransform w_T_obj(w_R_obj, w_p_obj);
        ode_r_obj.SetTransform(w_T_obj);
    }

    virtual void TearDown() {
    }

};

TEST_F(testODERigidObject, testVelocity)
{   
    Math3D::Vector3 w(0.31,0.06,0.14);
    Math3D::Vector3 v(2.3,5.1,6.5);
    ode_r_obj.SetVelocity(w, v);
    Math3D::Vector3 w_check, v_check;
    ode_r_obj.GetVelocity(w_check, v_check);
    ASSERT_TRUE(w_check == w);
    ASSERT_TRUE(v_check == v);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
