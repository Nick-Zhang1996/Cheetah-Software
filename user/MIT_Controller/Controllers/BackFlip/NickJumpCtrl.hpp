#ifndef NICKJUMP_CTRL
#define NICKJUMP_CTRL

#include "DataReader.hpp"
#include "DataReadCtrl.hpp"
#include <Dynamics/FloatingBaseModel.h>
#include <Controllers/LegController.h>

template <typename T>
class NickJumpCtrl : public DataReadCtrl<T> {
 public:
  NickJumpCtrl(DataReader*, float _dt);
  virtual ~NickJumpCtrl();

  virtual void OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command);

 protected:
  void _update_joint_command();
};

#endif
