#ifndef NICKJUMP_CTRL
#define NICKJUMP_CTRL

#include <cmath>
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
  using DataReadCtrl<T>::FirstVisit;
  void FirstVisit(float _curr_time){
    DataReadCtrl<T>::FirstVisit(_curr_time);
    _ready_for_landing = false;
    _leg_contact[0] = false;
    _leg_contact[1] = false;
    _leg_contact[2] = false;
    _leg_contact[3] = false;
    _contact_leg_count = 0;
  }
  bool EndOfPhase(LegControllerData<T>* data);

 protected:
  int _update_joint_command();
  // check leg contact, update _leg_contact and _contact_leg_count
  void _check_leg_contact(LegControllerData<T>* data);
  void _make_contact_leg_damper();
  // 1 0
  // 3 2
  // leg order
  bool _leg_contact[4] = {false};
  int _contact_leg_count;
  bool _ready_for_landing;
};

#endif
