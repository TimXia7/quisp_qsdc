#include "QSDCApplication.h"

using namespace omnetpp;
using namespace quisp::messages;

namespace quisp::modules {

void QSDCApplication::initialize() {
  initializeLogger(provider);

  // only keep this module for EndNodes (same logic as Application.cc)
  if (!gate("toRouter")->isConnected() || !gate("fromRouter")->isConnected()) {
    auto *msg = new DeleteThisModule("DeleteThisModule");
    scheduleAt(simTime(), msg);
    return;
  }

  my_address = provider.getNodeAddr();
  is_initiator = provider.getQNode()->par("is_initiator").boolValue();

  if (is_initiator) {
    // fire once (don't reschedule traffic)
    scheduleAt(simTime(), new cMessage("START_ONCE"));
  }
}

void QSDCApplication::handleMessage(cMessage* msg) {

  // If we receive the setup response, QRSA still needs it.
  if (auto *resp = dynamic_cast<ConnectionSetupResponse*>(msg)) {
    EV_INFO << "[QSDC] ConnectionSetupResponse received (ruleset_id="
            << resp->getRuleSet_id() << ")\n";

    startQSDCProtocol(resp->getRuleSet_id());

    delete resp;
    return;
  }

  if (dynamic_cast<DeleteThisModule*>(msg)) {
    delete msg;
    deleteModule();
    return;
  }

  if (strcmp(msg->getName(), "START_ONCE") == 0) {
    delete msg;
    startOnce();
    return;
  }

  // pass-through for other QRSA-related messages
  if (dynamic_cast<ConnectionSetupRequest*>(msg) ||
      dynamic_cast<InternalRuleSetForwarding*>(msg)) {
    logger->logPacket("QSDCApplication::handleMessage", msg);
    send(msg, "toRouter");
    return;
  }

  delete msg;
  error("QSDCApplication: unknown message");
}

void QSDCApplication::startOnce() {
  const int bob_addr = par("bob_addr").intValue();
  const int n_pairs  = par("number_of_bellpair").intValue();

  auto *pk = new ConnectionSetupRequest("ConnSetupRequest");
  pk->setApplicationId(0);

  pk->setActual_srcAddr(my_address);
  pk->setActual_destAddr(bob_addr);

  // routing headers (keep consistent with your earlier code)
  pk->setSrcAddr(my_address);
  pk->setDestAddr(my_address);

  // IMPORTANT: request stored Bell pairs (per your .msg generated API)
  pk->setNumber_of_required_Bellpairs(n_pairs);

  // IMPORTANT: do NOT request tomography measurements
  pk->setNum_measure(0);

  pk->setKind(7);

  EV_INFO << "[QSDC] Requesting " << n_pairs << " stored Bell pairs from "
          << my_address << " to " << bob_addr << "\n";

  send(pk, "toRouter");
}

void QSDCApplication::startQSDCProtocol(unsigned long ruleset_id) {
  if (!is_initiator) return;
  EV_INFO << "[QSDC] Starting QSDC operations (ruleset_id=" << ruleset_id << ")\n";
}

}  // namespace quisp::modules
