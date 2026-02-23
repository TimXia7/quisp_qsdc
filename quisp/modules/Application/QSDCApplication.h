#ifndef MODULES_QSDCAPPLICATION_H_
#define MODULES_QSDCAPPLICATION_H_

#include "IApplication.h"
#include "modules/Logger/LoggerBase.h"
#include "utils/ComponentProvider.h"

namespace quisp::modules {

class QSDCApplication : public IApplication, public Logger::LoggerBase {
 public:
  QSDCApplication() : provider(utils::ComponentProvider{this}) {}
  ~QSDCApplication() override {}

 protected:
  utils::ComponentProvider provider;
  int my_address = -1;
  bool is_initiator = false;

  void initialize() override;
  void handleMessage(omnetpp::cMessage* msg) override;

  void startOnce();
};

Define_Module(QSDCApplication);

}  // namespace quisp::modules

#endif
